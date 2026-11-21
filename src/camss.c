/*
 * camss.c - Qualcomm camss (RDI raw path) + Bayer sensor support for the
 * Tizen V4L2 camera HAL on the Radxa Dragon Q6A.
 *
 * The camss driver on QCS6490/SC7280 only exposes the RDI path: the sensor's
 * packed RAW10 Bayer frames land in memory unprocessed. This file sets up the
 * media pipeline (sensor -> csiphy -> csid0 -> vfe0_rdi0), captures RAW10 and
 * turns it into YUV on the CPU: black level, gray-world white balance,
 * bilinear demosaic, gamma, BT.601 conversion and scaling/cropping to the
 * requested size, plus a simple auto exposure loop driving the sensor's
 * exposure and analogue gain controls.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#include <linux/media.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <dlog.h>

#include "camss.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "CAMERA_HAL"

#define CAMSS_RAW_BUFFERS   4
#define CAMSS_SENSOR_W      1920
#define CAMSS_SENSOR_H      1080
#define CAMSS_BLACK_LEVEL   64
#define CAMSS_WHITE_LEVEL   1023
#define CAMSS_GAMMA_STEPS   4096
#define CAMSS_AE_PERIOD     3       /* frames between sensor control updates */
#define CAMSS_AE_TARGET     0.17    /* linear mean luminance target (~118 after gamma) */
#define CAMSS_SW_GAIN_MAX   8.0
#define CAMSS_AGAIN_MAX     8.0     /* IMX214: gain = 512 / (512 - code), code <= 448 */

/* entity name prefixes */
#define ENT_CSID            "msm_csid0"
#define ENT_RDI             "msm_vfe0_rdi0"
#define ENT_VIDEO           "msm_vfe0_video0"

enum { C_R = 0, C_G = 1, C_B = 2 };

static const char *g_sensor_prefixes[] = { "imx214", NULL };

typedef struct {
	void *data;
	size_t length;
} camss_raw_buffer;

struct camss_ctx {
	int media_fd;
	int sensor_fd;
	int csiphy_fd;
	int csid_fd;
	int rdi_fd;
	int video_fd;

	uint32_t mbus_code;
	uint32_t pix_fourcc;
	int pattern[2][2];          /* color at (y & 1, x & 1) */
	uint32_t width;
	uint32_t height;
	uint32_t bytesperline;

	camss_raw_buffer raw[CAMSS_RAW_BUFFERS];
	uint32_t raw_count;
	gboolean streaming;

	uint8_t *plane;             /* gamma-encoded, white-balanced mosaic, width x height */

	/* image processing state */
	uint8_t gamma[CAMSS_GAMMA_STEPS];
	uint8_t lut[3][1024];
	double wb_gain[3];
	double sw_gain;

	/* sensor exposure state */
	int exposure;
	int exposure_min;
	int exposure_max;
	int again_code;
	int again_code_max;
	double total;               /* exposure lines * analogue gain * sw gain */
	uint32_t frame_count;
};


/* ---------------------------------------------------------------- media */

static int __find_entity(int media_fd, const char *prefix, struct media_entity_desc *out)
{
	struct media_entity_desc e;

	memset(&e, 0, sizeof(e));
	e.id = MEDIA_ENT_ID_FLAG_NEXT;

	while (ioctl(media_fd, MEDIA_IOC_ENUM_ENTITIES, &e) == 0) {
		if (strncmp(e.name, prefix, strlen(prefix)) == 0) {
			*out = e;
			return 0;
		}
		e.id |= MEDIA_ENT_ID_FLAG_NEXT;
	}

	return -1;
}

static int __find_sensor(int media_fd, struct media_entity_desc *out)
{
	int i;

	for (i = 0 ; g_sensor_prefixes[i] ; i++) {
		if (__find_entity(media_fd, g_sensor_prefixes[i], out) == 0)
			return 0;
	}

	return -1;
}

/* Open the /dev node of an entity from its char device numbers. */
static int __open_entity_node(const struct media_entity_desc *e)
{
	char path[64];
	char line[128];
	char devname[64] = {0};
	FILE *fp;
	int fd;

	snprintf(path, sizeof(path), "/sys/dev/char/%u:%u/uevent", e->dev.major, e->dev.minor);

	fp = fopen(path, "r");
	if (!fp) {
		LOGE("no uevent for entity %s (%s)", e->name, path);
		return -1;
	}

	while (fgets(line, sizeof(line), fp)) {
		if (strncmp(line, "DEVNAME=", 8) == 0) {
			line[strcspn(line, "\n")] = '\0';
			snprintf(devname, sizeof(devname), "/dev/%s", line + 8);
			break;
		}
	}
	fclose(fp);

	if (!devname[0]) {
		LOGE("no DEVNAME for entity %s", e->name);
		return -1;
	}

	fd = open(devname, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		LOGE("open %s (%s) failed, errno %d", devname, e->name, errno);
	else
		LOGI("entity %s -> %s", e->name, devname);

	return fd;
}

/* Enable the first link from src to dst; *src_pad gets the source pad index. */
static int __enable_link(int media_fd, const struct media_entity_desc *src,
	const struct media_entity_desc *dst, uint32_t *src_pad)
{
	struct media_links_enum le;
	struct media_pad_desc *pads;
	struct media_link_desc *links;
	uint32_t i;
	int ret = -1;

	pads = g_new0(struct media_pad_desc, src->pads ? src->pads : 1);
	links = g_new0(struct media_link_desc, src->links ? src->links : 1);

	memset(&le, 0, sizeof(le));
	le.entity = src->id;
	le.pads = pads;
	le.links = links;

	if (ioctl(media_fd, MEDIA_IOC_ENUM_LINKS, &le) < 0) {
		LOGE("ENUM_LINKS %s failed, errno %d", src->name, errno);
		goto out;
	}

	for (i = 0 ; i < src->links ; i++) {
		if (links[i].source.entity != src->id || links[i].sink.entity != dst->id)
			continue;

		if (src_pad)
			*src_pad = links[i].source.index;

		if (links[i].flags & MEDIA_LNK_FL_ENABLED) {
			ret = 0;
			goto out;
		}

		links[i].flags |= MEDIA_LNK_FL_ENABLED;
		if (ioctl(media_fd, MEDIA_IOC_SETUP_LINK, &links[i]) < 0) {
			LOGE("SETUP_LINK %s -> %s failed, errno %d", src->name, dst->name, errno);
			goto out;
		}

		LOGI("link %s:%u -> %s:%u enabled", src->name,
			links[i].source.index, dst->name, links[i].sink.index);
		ret = 0;
		goto out;
	}

	LOGE("no link %s -> %s", src->name, dst->name);

out:
	g_free(pads);
	g_free(links);
	return ret;
}

/* Find the entity the sensor's source pad is linked to (the csiphy). */
static int __find_sensor_peer(int media_fd, const struct media_entity_desc *sensor,
	struct media_entity_desc *peer)
{
	struct media_links_enum le;
	struct media_pad_desc *pads;
	struct media_link_desc *links;
	uint32_t i;
	int ret = -1;

	pads = g_new0(struct media_pad_desc, sensor->pads ? sensor->pads : 1);
	links = g_new0(struct media_link_desc, sensor->links ? sensor->links : 1);

	memset(&le, 0, sizeof(le));
	le.entity = sensor->id;
	le.pads = pads;
	le.links = links;

	if (ioctl(media_fd, MEDIA_IOC_ENUM_LINKS, &le) == 0) {
		for (i = 0 ; i < sensor->links ; i++) {
			if (links[i].source.entity != sensor->id)
				continue;

			memset(peer, 0, sizeof(*peer));
			peer->id = links[i].sink.entity;
			if (ioctl(media_fd, MEDIA_IOC_ENUM_ENTITIES, peer) == 0) {
				ret = 0;
				break;
			}
		}
	}

	g_free(pads);
	g_free(links);
	return ret;
}

static int __subdev_set_fmt(int fd, uint32_t pad, struct v4l2_mbus_framefmt *fmt, gboolean must)
{
	struct v4l2_subdev_format sf;

	memset(&sf, 0, sizeof(sf));
	sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sf.pad = pad;
	sf.format = *fmt;

	if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sf) < 0) {
		if (must)
			LOGE("SUBDEV_S_FMT pad %u failed, errno %d", pad, errno);
		return must ? -1 : 0;
	}

	if (must)
		*fmt = sf.format;

	return 0;
}

static int __set_ctrl(int fd, uint32_t id, int value)
{
	struct v4l2_control c;

	memset(&c, 0, sizeof(c));
	c.id = id;
	c.value = value;

	return ioctl(fd, VIDIOC_S_CTRL, &c);
}

static int __query_ctrl(int fd, uint32_t id, int *min, int *max, int *def)
{
	struct v4l2_queryctrl q;

	memset(&q, 0, sizeof(q));
	q.id = id;

	if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0)
		return -1;

	if (min)
		*min = q.minimum;
	if (max)
		*max = q.maximum;
	if (def)
		*def = q.default_value;

	return 0;
}

static int __open_media_with_sensor(struct media_entity_desc *sensor)
{
	glob_t g;
	size_t i;
	int fd = -1;

	memset(&g, 0, sizeof(g));
	if (glob("/dev/media*", 0, NULL, &g) != 0)
		return -1;

	for (i = 0 ; i < g.gl_pathc ; i++) {
		fd = open(g.gl_pathv[i], O_RDWR | O_CLOEXEC);
		if (fd < 0)
			continue;

		if (__find_sensor(fd, sensor) == 0) {
			LOGI("sensor [%s] on %s", sensor->name, g.gl_pathv[i]);
			break;
		}

		close(fd);
		fd = -1;
	}

	globfree(&g);

	return fd;
}


/* ---------------------------------------------------------------- public: info */

gboolean camss_detect(void)
{
	struct media_entity_desc sensor;
	int fd = __open_media_with_sensor(&sensor);

	if (fd < 0)
		return FALSE;

	close(fd);
	return TRUE;
}

void camss_fill_device_info(camera_device_info_s *info, int index)
{
	static const camera_resolution_s res[] = {
		{1920, 1080}, {1280, 720}, {640, 480}, {320, 240}
	};
	static const camera_pixel_format_e formats[] = {
		CAMERA_PIXEL_FORMAT_I420, CAMERA_PIXEL_FORMAT_NV12, CAMERA_PIXEL_FORMAT_YUYV
	};
	uint32_t i;

	memset(info, 0, sizeof(*info));

	info->index = index;
	snprintf(info->name, sizeof(info->name), "IMX214");
	snprintf(info->node_path, sizeof(info->node_path), "camss");
	info->facing_direction = CAMERA_FACING_DIRECTION_REAR;

	for (i = 0 ; i < G_N_ELEMENTS(formats) ; i++)
		info->format_list.formats[i] = formats[i];
	info->format_list.count = G_N_ELEMENTS(formats);

	for (i = 0 ; i < G_N_ELEMENTS(res) ; i++) {
		info->preview_list.resolutions[i] = res[i];
		info->capture_list.resolutions[i] = res[i];
		info->video_list.resolutions[i] = res[i];

		info->preview_fps_list[i].fps[0] = 30;
		info->preview_fps_list[i].fps[1] = 15;
		info->preview_fps_list[i].count = 2;
		info->video_fps_list[i] = info->preview_fps_list[i];
	}
	info->preview_list.count = G_N_ELEMENTS(res);
	info->capture_list.count = G_N_ELEMENTS(res);
	info->video_list.count = G_N_ELEMENTS(res);
}

uint32_t camss_frame_size(camera_pixel_format_e format, uint32_t width, uint32_t height)
{
	switch (format) {
	case CAMERA_PIXEL_FORMAT_NV12:
	case CAMERA_PIXEL_FORMAT_NV21:
	case CAMERA_PIXEL_FORMAT_I420:
	case CAMERA_PIXEL_FORMAT_YV12:
		return width * height * 3 / 2;
	case CAMERA_PIXEL_FORMAT_YUYV:
	case CAMERA_PIXEL_FORMAT_UYVY:
		return width * height * 2;
	default:
		return 0;
	}
}


/* ---------------------------------------------------------------- image processing */

static void __build_gamma(camss_ctx *ctx)
{
	int i;

	for (i = 0 ; i < CAMSS_GAMMA_STEPS ; i++) {
		double x = (double)i / (CAMSS_GAMMA_STEPS - 1);
		/* sRGB-like curve: linear toe, 1/2.4 power above */
		double y = (x <= 0.0031308) ? 12.92 * x : 1.055 * pow(x, 1.0 / 2.4) - 0.055;
		ctx->gamma[i] = (uint8_t)(y * 255.0 + 0.5);
	}
}

static void __build_lut(camss_ctx *ctx)
{
	int c, v;
	const double range = CAMSS_WHITE_LEVEL - CAMSS_BLACK_LEVEL;

	for (c = 0 ; c < 3 ; c++) {
		double scale = ctx->wb_gain[c] * ctx->sw_gain * (CAMSS_GAMMA_STEPS - 1) / range;

		for (v = 0 ; v < 1024 ; v++) {
			double lin = (v - CAMSS_BLACK_LEVEL) * scale;
			int idx = lin < 0 ? 0 : (lin > CAMSS_GAMMA_STEPS - 1 ? CAMSS_GAMMA_STEPS - 1 : (int)lin);
			ctx->lut[c][v] = ctx->gamma[idx];
		}
	}
}

static void __set_pattern(camss_ctx *ctx)
{
	switch (ctx->mbus_code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		ctx->pattern[0][0] = C_G; ctx->pattern[0][1] = C_R;
		ctx->pattern[1][0] = C_B; ctx->pattern[1][1] = C_G;
		ctx->pix_fourcc = V4L2_PIX_FMT_SGRBG10P;
		break;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		ctx->pattern[0][0] = C_G; ctx->pattern[0][1] = C_B;
		ctx->pattern[1][0] = C_R; ctx->pattern[1][1] = C_G;
		ctx->pix_fourcc = V4L2_PIX_FMT_SGBRG10P;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		ctx->pattern[0][0] = C_B; ctx->pattern[0][1] = C_G;
		ctx->pattern[1][0] = C_G; ctx->pattern[1][1] = C_R;
		ctx->pix_fourcc = V4L2_PIX_FMT_SBGGR10P;
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	default:
		ctx->pattern[0][0] = C_R; ctx->pattern[0][1] = C_G;
		ctx->pattern[1][0] = C_G; ctx->pattern[1][1] = C_B;
		ctx->pix_fourcc = V4L2_PIX_FMT_SRGGB10P;
		break;
	}
}

/*
 * Unpack RAW10 into ctx->plane through the per-channel LUT and gather linear
 * per-channel statistics on a sparse grid.
 */
static void __unpack(camss_ctx *ctx, const uint8_t *raw, double sum[3], uint32_t cnt[3], uint32_t *saturated)
{
	uint32_t x, y;
	const uint32_t w = ctx->width;

	for (y = 0 ; y < ctx->height ; y++) {
		const uint8_t *src = raw + (size_t)y * ctx->bytesperline;
		uint8_t *dst = ctx->plane + (size_t)y * w;
		const uint8_t *lut0 = ctx->lut[ctx->pattern[y & 1][0]];
		const uint8_t *lut1 = ctx->lut[ctx->pattern[y & 1][1]];
		gboolean stats_row = ((y & 7) < 2);

		for (x = 0 ; x < w ; x += 4, src += 5, dst += 4) {
			uint32_t lo = src[4];
			uint32_t p0 = ((uint32_t)src[0] << 2) | (lo & 3);
			uint32_t p1 = ((uint32_t)src[1] << 2) | ((lo >> 2) & 3);
			uint32_t p2 = ((uint32_t)src[2] << 2) | ((lo >> 4) & 3);
			uint32_t p3 = ((uint32_t)src[3] << 2) | ((lo >> 6) & 3);

			dst[0] = lut0[p0];
			dst[1] = lut1[p1];
			dst[2] = lut0[p2];
			dst[3] = lut1[p3];

			if (stats_row && (x & 15) == 0) {
				int c0 = ctx->pattern[y & 1][0];
				int c1 = ctx->pattern[y & 1][1];

				sum[c0] += (double)p0 - CAMSS_BLACK_LEVEL;
				sum[c1] += (double)p1 - CAMSS_BLACK_LEVEL;
				cnt[c0]++;
				cnt[c1]++;
				if (p0 >= CAMSS_WHITE_LEVEL - 8 || p1 >= CAMSS_WHITE_LEVEL - 8)
					(*saturated)++;
			}
		}
	}
}

/* Bilinear demosaic at (x, y) of the gamma-encoded mosaic. */
static inline void __rgb_at(const camss_ctx *ctx, int x, int y, int *r, int *g, int *b)
{
	const int w = (int)ctx->width;
	const uint8_t *p;
	int c, hc;
	int v, hv, vv, dv;

	/* keep the neighbours inside the frame; step by 2 to keep the Bayer phase */
	if (x < 1)
		x += 2;
	else if (x > w - 2)
		x -= 2;
	if (y < 1)
		y += 2;
	else if (y > (int)ctx->height - 2)
		y -= 2;

	p = ctx->plane + (size_t)y * w + x;
	c = ctx->pattern[y & 1][x & 1];

	v = p[0];
	hv = (p[-1] + p[1] + 1) >> 1;
	vv = (p[-w] + p[w] + 1) >> 1;

	if (c == C_G) {
		hc = ctx->pattern[y & 1][(x + 1) & 1];
		*g = v;
		if (hc == C_R) {
			*r = hv;
			*b = vv;
		} else {
			*b = hv;
			*r = vv;
		}
		return;
	}

	dv = (p[-w - 1] + p[-w + 1] + p[w - 1] + p[w + 1] + 2) >> 2;
	*g = (hv + vv + 1) >> 1;
	if (c == C_R) {
		*r = v;
		*b = dv;
	} else {
		*b = v;
		*r = dv;
	}
}

static inline uint8_t __clip(int v)
{
	return v < 0 ? 0 : (v > 255 ? 255 : v);
}

/* BT.601 limited range */
static inline uint8_t __y(int r, int g, int b)
{
	return __clip(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
}

static inline uint8_t __u(int r, int g, int b)
{
	return __clip(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
}

static inline uint8_t __v(int r, int g, int b)
{
	return __clip(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
}

/* Convert ctx->plane into out at out->resolution, centre-cropped to the output aspect ratio. */
static void __convert(camss_ctx *ctx, camera_buffer_s *out)
{
	const uint32_t ow = out->resolution.width;
	const uint32_t oh = out->resolution.height;
	uint32_t cw = ctx->width;
	uint32_t ch = ctx->height;
	uint32_t cx, cy, ox, oy;
	uint8_t *base = out->planes[0].data;
	uint8_t *yp = base;
	uint8_t *up = NULL;
	uint8_t *vp = NULL;
	int uv_step = 1;
	gboolean packed = FALSE;
	int u_off = 1, v_off = 3, y_off = 0;

	/* crop the sensor frame to the output aspect ratio */
	if ((uint64_t)cw * oh > (uint64_t)ch * ow)
		cw = (uint32_t)((uint64_t)ch * ow / oh);
	else
		ch = (uint32_t)((uint64_t)cw * oh / ow);
	cx = ((ctx->width - cw) / 2) & ~1u;
	cy = ((ctx->height - ch) / 2) & ~1u;

	switch (out->format) {
	case CAMERA_PIXEL_FORMAT_NV12:
		up = base + ow * oh;
		vp = up + 1;
		uv_step = 2;
		break;
	case CAMERA_PIXEL_FORMAT_NV21:
		vp = base + ow * oh;
		up = vp + 1;
		uv_step = 2;
		break;
	case CAMERA_PIXEL_FORMAT_I420:
		up = base + ow * oh;
		vp = up + (ow / 2) * (oh / 2);
		break;
	case CAMERA_PIXEL_FORMAT_YV12:
		vp = base + ow * oh;
		up = vp + (ow / 2) * (oh / 2);
		break;
	case CAMERA_PIXEL_FORMAT_YUYV:
		packed = TRUE;
		y_off = 0; u_off = 1; v_off = 3;
		break;
	case CAMERA_PIXEL_FORMAT_UYVY:
	default:
		packed = TRUE;
		y_off = 1; u_off = 0; v_off = 2;
		break;
	}

	for (oy = 0 ; oy < oh ; oy += 2) {
		int sy0 = (int)(cy + (uint64_t)oy * ch / oh);
		int sy1 = (int)(cy + (uint64_t)(oy + 1) * ch / oh);

		for (ox = 0 ; ox < ow ; ox += 2) {
			int sx0 = (int)(cx + (uint64_t)ox * cw / ow);
			int sx1 = (int)(cx + (uint64_t)(ox + 1) * cw / ow);
			int r[4], g[4], b[4];
			int rs, gs, bs;
			uint8_t yv[4], uu, vv;

			__rgb_at(ctx, sx0, sy0, &r[0], &g[0], &b[0]);
			__rgb_at(ctx, sx1, sy0, &r[1], &g[1], &b[1]);
			__rgb_at(ctx, sx0, sy1, &r[2], &g[2], &b[2]);
			__rgb_at(ctx, sx1, sy1, &r[3], &g[3], &b[3]);

			yv[0] = __y(r[0], g[0], b[0]);
			yv[1] = __y(r[1], g[1], b[1]);
			yv[2] = __y(r[2], g[2], b[2]);
			yv[3] = __y(r[3], g[3], b[3]);

			rs = (r[0] + r[1] + r[2] + r[3] + 2) >> 2;
			gs = (g[0] + g[1] + g[2] + g[3] + 2) >> 2;
			bs = (b[0] + b[1] + b[2] + b[3] + 2) >> 2;
			uu = __u(rs, gs, bs);
			vv = __v(rs, gs, bs);

			if (packed) {
				uint8_t *l0 = base + ((size_t)oy * ow + ox) * 2;
				uint8_t *l1 = l0 + (size_t)ow * 2;

				l0[y_off] = yv[0]; l0[y_off + 2] = yv[1];
				l0[u_off] = uu; l0[v_off] = vv;
				l1[y_off] = yv[2]; l1[y_off + 2] = yv[3];
				l1[u_off] = uu; l1[v_off] = vv;
			} else {
				size_t ci = (size_t)(oy / 2) * (ow / 2) + ox / 2;

				yp[(size_t)oy * ow + ox] = yv[0];
				yp[(size_t)oy * ow + ox + 1] = yv[1];
				yp[(size_t)(oy + 1) * ow + ox] = yv[2];
				yp[(size_t)(oy + 1) * ow + ox + 1] = yv[3];
				up[ci * uv_step] = uu;
				vp[ci * uv_step] = vv;
			}
		}
	}
}


/* ---------------------------------------------------------------- 3A */

static int __again_code(double gain)
{
	int code;

	if (gain < 1.0)
		gain = 1.0;
	code = (int)(512.0 - 512.0 / gain + 0.5);

	return code;
}

static double __again_value(int code)
{
	return 512.0 / (512.0 - code);
}

/* Split the total exposure into sensor exposure lines, analogue gain and software gain. */
static void __apply_total(camss_ctx *ctx)
{
	double t = ctx->total;
	double again_max = __again_value(ctx->again_code_max);
	double again, sw;
	int lines, code;

	if (t < ctx->exposure_min)
		t = ctx->exposure_min;

	if (t <= ctx->exposure_max) {
		lines = (int)t;
		code = 0;
		sw = 1.0;
	} else {
		lines = ctx->exposure_max;
		again = t / lines;
		if (again > again_max) {
			code = ctx->again_code_max;
			sw = (again / again_max);
			if (sw > CAMSS_SW_GAIN_MAX)
				sw = CAMSS_SW_GAIN_MAX;
		} else {
			code = __again_code(again);
			if (code > ctx->again_code_max)
				code = ctx->again_code_max;
			sw = again / __again_value(code);
		}
	}

	ctx->total = lines * __again_value(code) * sw;

	if (lines != ctx->exposure) {
		if (__set_ctrl(ctx->sensor_fd, V4L2_CID_EXPOSURE, lines) == 0)
			ctx->exposure = lines;
	}

	if (code != ctx->again_code) {
		if (__set_ctrl(ctx->sensor_fd, V4L2_CID_ANALOGUE_GAIN, code) == 0)
			ctx->again_code = code;
	}

	ctx->sw_gain = sw;
}

static void __run_3a(camss_ctx *ctx, const double sum[3], const uint32_t cnt[3], uint32_t saturated)
{
	double mean[3];
	double luma, ratio, sat_frac;
	uint32_t total_cnt = cnt[0] + cnt[1] + cnt[2];
	int c;

	if (!cnt[0] || !cnt[1] || !cnt[2])
		return;

	for (c = 0 ; c < 3 ; c++) {
		mean[c] = sum[c] / cnt[c];
		if (mean[c] < 0.5)
			mean[c] = 0.5;
	}

	/* gray-world white balance, smoothed */
	{
		double gr = mean[C_G] / mean[C_R];
		double gb = mean[C_G] / mean[C_B];

		gr = CLAMP(gr, 0.5, 4.0);
		gb = CLAMP(gb, 0.5, 4.0);
		ctx->wb_gain[C_R] = ctx->wb_gain[C_R] * 0.8 + gr * 0.2;
		ctx->wb_gain[C_B] = ctx->wb_gain[C_B] * 0.8 + gb * 0.2;
		ctx->wb_gain[C_G] = 1.0;
	}

	if (++ctx->frame_count % CAMSS_AE_PERIOD == 0) {
		/* luminance at the current software gain, normalized */
		luma = (0.299 * mean[C_R] * ctx->wb_gain[C_R] + 0.587 * mean[C_G] +
			0.114 * mean[C_B] * ctx->wb_gain[C_B]) * ctx->sw_gain /
			(CAMSS_WHITE_LEVEL - CAMSS_BLACK_LEVEL);
		if (luma < 1e-4)
			luma = 1e-4;

		ratio = CAMSS_AE_TARGET / luma;
		sat_frac = (double)saturated / (total_cnt / 2.0);
		if (sat_frac > 0.05 && ratio > 0.8)
			ratio = 0.8;

		/* damp and limit each step */
		ratio = pow(ratio, 0.7);
		ratio = CLAMP(ratio, 0.33, 3.0);

		if (ratio < 0.97 || ratio > 1.03) {
			ctx->total *= ratio;
			__apply_total(ctx);
		}

		if (ctx->frame_count % (CAMSS_AE_PERIOD * 30) == 0)
			LOGI("3A: luma %.3f exp %d again %d (%.2fx) sw %.2f wb %.2f/%.2f sat %.3f",
				luma, ctx->exposure, ctx->again_code, __again_value(ctx->again_code),
				ctx->sw_gain, ctx->wb_gain[C_R], ctx->wb_gain[C_B], sat_frac);
	}

	__build_lut(ctx);
}


/* ---------------------------------------------------------------- public: stream */

camss_ctx *camss_open(int *error)
{
	struct media_entity_desc sensor, phy, csid, rdi, video;
	camss_ctx *ctx;
	uint32_t pad = 0;

	ctx = g_new0(camss_ctx, 1);
	ctx->media_fd = ctx->sensor_fd = ctx->csiphy_fd = -1;
	ctx->csid_fd = ctx->rdi_fd = ctx->video_fd = -1;

	*error = CAMERA_ERROR_DEVICE_OPEN;

	ctx->media_fd = __open_media_with_sensor(&sensor);
	if (ctx->media_fd < 0) {
		LOGE("no camss sensor");
		*error = CAMERA_ERROR_DEVICE_NOT_FOUND;
		goto fail;
	}

	if (__find_sensor_peer(ctx->media_fd, &sensor, &phy) < 0 ||
		__find_entity(ctx->media_fd, ENT_CSID, &csid) < 0 ||
		__find_entity(ctx->media_fd, ENT_RDI, &rdi) < 0 ||
		__find_entity(ctx->media_fd, ENT_VIDEO, &video) < 0) {
		LOGE("camss entities not found");
		goto fail;
	}

	LOGI("pipeline: %s -> %s -> %s -> %s -> %s",
		sensor.name, phy.name, csid.name, rdi.name, video.name);

	if (__enable_link(ctx->media_fd, &sensor, &phy, NULL) < 0 ||
		__enable_link(ctx->media_fd, &phy, &csid, NULL) < 0 ||
		__enable_link(ctx->media_fd, &csid, &rdi, &pad) < 0 ||
		__enable_link(ctx->media_fd, &rdi, &video, NULL) < 0)
		goto fail;

	ctx->sensor_fd = __open_entity_node(&sensor);
	ctx->csiphy_fd = __open_entity_node(&phy);
	ctx->csid_fd = __open_entity_node(&csid);
	ctx->rdi_fd = __open_entity_node(&rdi);
	ctx->video_fd = __open_entity_node(&video);

	if (ctx->sensor_fd < 0 || ctx->csiphy_fd < 0 || ctx->csid_fd < 0 ||
		ctx->rdi_fd < 0 || ctx->video_fd < 0) {
		*error = (errno == EACCES || errno == EPERM) ?
			CAMERA_ERROR_PERMISSION_DENIED : CAMERA_ERROR_DEVICE_OPEN;
		goto fail;
	}

	/* exposure limits */
	if (__query_ctrl(ctx->sensor_fd, V4L2_CID_EXPOSURE, &ctx->exposure_min, &ctx->exposure_max, NULL) < 0) {
		ctx->exposure_min = 1;
		ctx->exposure_max = 1000;
	}
	if (__query_ctrl(ctx->sensor_fd, V4L2_CID_ANALOGUE_GAIN, NULL, &ctx->again_code_max, NULL) < 0)
		ctx->again_code_max = 0;
	if (ctx->again_code_max > __again_code(CAMSS_AGAIN_MAX))
		ctx->again_code_max = __again_code(CAMSS_AGAIN_MAX);

	__build_gamma(ctx);
	ctx->wb_gain[C_R] = 1.8;
	ctx->wb_gain[C_G] = 1.0;
	ctx->wb_gain[C_B] = 1.6;
	ctx->sw_gain = 1.0;
	ctx->exposure = -1;
	ctx->again_code = -1;

	*error = CAMERA_ERROR_NONE;
	return ctx;

fail:
	camss_close(ctx);
	return NULL;
}

void camss_close(camss_ctx *ctx)
{
	if (!ctx)
		return;

	if (ctx->streaming)
		camss_stop(ctx);

	if (ctx->video_fd >= 0)
		close(ctx->video_fd);
	if (ctx->rdi_fd >= 0)
		close(ctx->rdi_fd);
	if (ctx->csid_fd >= 0)
		close(ctx->csid_fd);
	if (ctx->csiphy_fd >= 0)
		close(ctx->csiphy_fd);
	if (ctx->sensor_fd >= 0)
		close(ctx->sensor_fd);
	if (ctx->media_fd >= 0)
		close(ctx->media_fd);

	g_free(ctx->plane);
	g_free(ctx);
}

int camss_get_fd(camss_ctx *ctx)
{
	return ctx ? ctx->video_fd : -1;
}

int camss_start(camss_ctx *ctx, uint32_t fps)
{
	struct v4l2_subdev_format sf;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_format vf;
	struct v4l2_requestbuffers rb;
	uint32_t i;
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	if (!ctx)
		return CAMERA_ERROR_INVALID_PARAMETER;

	/* sensor mode */
	memset(&sf, 0, sizeof(sf));
	sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sf.pad = 0;
	if (ioctl(ctx->sensor_fd, VIDIOC_SUBDEV_G_FMT, &sf) < 0) {
		LOGE("sensor G_FMT failed, errno %d", errno);
		return CAMERA_ERROR_INTERNAL;
	}
	sf.format.width = CAMSS_SENSOR_W;
	sf.format.height = CAMSS_SENSOR_H;
	if (ioctl(ctx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &sf) < 0) {
		LOGE("sensor S_FMT failed, errno %d", errno);
		return CAMERA_ERROR_INTERNAL;
	}
	fmt = sf.format;
	ctx->mbus_code = fmt.code;
	__set_pattern(ctx);

	LOGI("sensor format 0x%04x %ux%u, fps %u", fmt.code, fmt.width, fmt.height, fps);

	if (__subdev_set_fmt(ctx->csiphy_fd, 0, &fmt, TRUE) < 0 ||
		__subdev_set_fmt(ctx->csiphy_fd, 1, &fmt, FALSE) < 0 ||
		__subdev_set_fmt(ctx->csid_fd, 0, &fmt, TRUE) < 0 ||
		__subdev_set_fmt(ctx->csid_fd, 1, &fmt, FALSE) < 0 ||
		__subdev_set_fmt(ctx->rdi_fd, 0, &fmt, TRUE) < 0 ||
		__subdev_set_fmt(ctx->rdi_fd, 1, &fmt, FALSE) < 0)
		return CAMERA_ERROR_INTERNAL;

	/* capture node */
	memset(&vf, 0, sizeof(vf));
	vf.type = type;
	vf.fmt.pix_mp.width = fmt.width;
	vf.fmt.pix_mp.height = fmt.height;
	vf.fmt.pix_mp.pixelformat = ctx->pix_fourcc;
	vf.fmt.pix_mp.num_planes = 1;
	vf.fmt.pix_mp.field = V4L2_FIELD_NONE;
	if (ioctl(ctx->video_fd, VIDIOC_S_FMT, &vf) < 0) {
		LOGE("video S_FMT failed, errno %d", errno);
		return CAMERA_ERROR_INTERNAL;
	}

	ctx->width = vf.fmt.pix_mp.width;
	ctx->height = vf.fmt.pix_mp.height;
	ctx->bytesperline = vf.fmt.pix_mp.plane_fmt[0].bytesperline;

	LOGI("raw %ux%u bpl %u", ctx->width, ctx->height, ctx->bytesperline);

	g_free(ctx->plane);
	ctx->plane = g_malloc((size_t)ctx->width * ctx->height);

	memset(&rb, 0, sizeof(rb));
	rb.count = CAMSS_RAW_BUFFERS;
	rb.type = type;
	rb.memory = V4L2_MEMORY_MMAP;
	if (ioctl(ctx->video_fd, VIDIOC_REQBUFS, &rb) < 0 || rb.count == 0) {
		LOGE("REQBUFS failed, errno %d", errno);
		return CAMERA_ERROR_INTERNAL;
	}
	ctx->raw_count = MIN(rb.count, CAMSS_RAW_BUFFERS);

	for (i = 0 ; i < ctx->raw_count ; i++) {
		struct v4l2_plane pl[1];
		struct v4l2_buffer b;

		memset(pl, 0, sizeof(pl));
		memset(&b, 0, sizeof(b));
		b.type = type;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		b.m.planes = pl;
		b.length = 1;

		if (ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &b) < 0) {
			LOGE("QUERYBUF %u failed, errno %d", i, errno);
			goto fail;
		}

		ctx->raw[i].length = pl[0].length;
		ctx->raw[i].data = mmap(NULL, pl[0].length, PROT_READ | PROT_WRITE,
			MAP_SHARED, ctx->video_fd, pl[0].m.mem_offset);
		if (ctx->raw[i].data == MAP_FAILED) {
			ctx->raw[i].data = NULL;
			LOGE("mmap %u failed, errno %d", i, errno);
			goto fail;
		}

		if (ioctl(ctx->video_fd, VIDIOC_QBUF, &b) < 0) {
			LOGE("QBUF %u failed, errno %d", i, errno);
			goto fail;
		}
	}

	/* initial exposure: bright enough for indoor light, AE takes it from there */
	if (ctx->total <= 0.0)
		ctx->total = ctx->exposure_max * 4.0;
	__apply_total(ctx);
	__build_lut(ctx);

	if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &type) < 0) {
		LOGE("STREAMON failed, errno %d", errno);
		goto fail;
	}

	ctx->streaming = TRUE;
	ctx->frame_count = 0;

	LOGI("camss stream on");

	return CAMERA_ERROR_NONE;

fail:
	camss_stop(ctx);
	return CAMERA_ERROR_INTERNAL;
}

int camss_stop(camss_ctx *ctx)
{
	struct v4l2_requestbuffers rb;
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	uint32_t i;

	if (!ctx)
		return CAMERA_ERROR_INVALID_PARAMETER;

	if (ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type) < 0)
		LOGW("STREAMOFF failed, errno %d", errno);

	for (i = 0 ; i < CAMSS_RAW_BUFFERS ; i++) {
		if (ctx->raw[i].data) {
			munmap(ctx->raw[i].data, ctx->raw[i].length);
			ctx->raw[i].data = NULL;
			ctx->raw[i].length = 0;
		}
	}

	memset(&rb, 0, sizeof(rb));
	rb.count = 0;
	rb.type = type;
	rb.memory = V4L2_MEMORY_MMAP;
	if (ioctl(ctx->video_fd, VIDIOC_REQBUFS, &rb) < 0)
		LOGW("REQBUFS 0 failed, errno %d", errno);

	ctx->raw_count = 0;
	ctx->streaming = FALSE;

	LOGI("camss stream off");

	return CAMERA_ERROR_NONE;
}

int camss_get_frame(camss_ctx *ctx, camera_buffer_s *out)
{
	struct v4l2_plane pl[1];
	struct v4l2_buffer b;
	double sum[3] = {0, 0, 0};
	uint32_t cnt[3] = {0, 0, 0};
	uint32_t saturated = 0;
	uint32_t size;

	if (!ctx || !ctx->streaming)
		return CAMERA_ERROR_INVALID_STATE;

	memset(pl, 0, sizeof(pl));
	memset(&b, 0, sizeof(b));
	b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b.memory = V4L2_MEMORY_MMAP;
	b.m.planes = pl;
	b.length = 1;

	if (ioctl(ctx->video_fd, VIDIOC_DQBUF, &b) < 0) {
		LOGE("DQBUF failed, errno %d", errno);
		return CAMERA_ERROR_DEVICE_READ;
	}

	if (b.index >= ctx->raw_count || !ctx->raw[b.index].data) {
		LOGE("bad buffer index %u", b.index);
		return CAMERA_ERROR_DEVICE_READ;
	}

	__unpack(ctx, ctx->raw[b.index].data, sum, cnt, &saturated);

	if (ioctl(ctx->video_fd, VIDIOC_QBUF, &b) < 0)
		LOGE("QBUF %u failed, errno %d", b.index, errno);

	if (out) {
		size = camss_frame_size(out->format, out->resolution.width, out->resolution.height);
		if (size && out->planes[0].data && out->planes[0].size >= size) {
			__convert(ctx, out);
			out->planes[0].bytesused = size;
		} else {
			LOGE("output buffer too small or bad format %d", out->format);
		}
	}

	__run_3a(ctx, sum, cnt, saturated);

	return CAMERA_ERROR_NONE;
}
