/*
 * hal_backend_tizen_camera_v4l2.c
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <glob.h>
#include <dlog.h>
#include <sched.h>
#include "hal_backend_camera_v4l2_private.h"
#include <tbm_surface.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif /* LOG_TAG */
#define LOG_TAG "CAMERA_HAL"

#define ENV_VIRTUAL_CAMERA      "VIRTUAL_CAMERA"
#define VIRTUAL_CAMERA_FMT_MAX  2
#define VIRTUAL_CAMERA_RES_MAX  3
#define TEST_JPEG_PATH          "/home/owner/media/Images/test.jpg"
#define DEVICE_NODE_PATH_MAX    16
#define DEVICE_NODE_PATH_PREFIX "/dev/video"
#define FOURCC_FORMAT           "%c%c%c%c"
#define FOURCC_CONVERT(fourcc)  \
	fourcc & 0xff,\
	(fourcc >> 8) & 0xff,\
	(fourcc >> 16) & 0xff,\
	(fourcc >> 24) & 0xff


static gboolean g_is_virtual_camera_mode;
static uint32_t g_virtual_camera_bytesused;
static gulong g_virtual_camera_frame_delay;
static camera_device_info_list_s *g_device_info_list;
static guint32 g_device_caps;
static GMutex g_device_info_lock;
static uint32_t g_virtual_camera_format[VIRTUAL_CAMERA_FMT_MAX] = {
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_NV12
};
static camera_resolution_s g_virtual_camera_res[VIRTUAL_CAMERA_RES_MAX] = {
	{640, 480},
	{1280, 720},
	{1920, 1080}
};


static void __camera_hal_v4l2_destructor(void) __attribute__((destructor));
static void __camera_send_message(hal_camera_handle *handle, camera_message_type_e type, int value);


static void __camera_hal_v4l2_destructor(void)
{
	LOGD("release device info list %p", g_device_info_list);

	g_free(g_device_info_list);
	g_device_info_list = NULL;

	return;
}


static void __camera_send_message(hal_camera_handle *handle, camera_message_type_e type, int value)
{
	camera_message_s *message = NULL;

	if (!handle) {
		LOGE("NULL handle");
		return;
	}

	message = g_new0(camera_message_s, 1);

	message->type = type;

	switch (type) {
	case CAMERA_MESSAGE_TYPE_FOCUS_CHANGED:
		message->focus_state = (camera_focus_state_e)value;
		break;
	case CAMERA_MESSAGE_TYPE_CAPTURED:
		break;
	case CAMERA_MESSAGE_TYPE_HDR_PROGRESS:
		message->hdr_progress = (uint32_t)value;
		break;
	case CAMERA_MESSAGE_TYPE_ERROR:
		message->error_code = (camera_error_e)value;
		break;
	default:
		LOGE("unknown type[%d]", type);
		g_free(message);
		return;
	}

	g_mutex_lock(&handle->msg_cb_lock);

	LOGD("type[%d], value[0x%x]", type, value);

	g_queue_push_tail(handle->msg_list, message);
	g_cond_signal(&handle->msg_cb_cond);

	g_mutex_unlock(&handle->msg_cb_lock);
}


static int __camera_v4l2_wait_frame(int device_fd, int wait_time)
{
	int ret = CAMERA_ERROR_NONE;
	fd_set fds;
	struct timeval timeout;

	if (g_is_virtual_camera_mode) {
		g_usleep(g_virtual_camera_frame_delay);
		return CAMERA_ERROR_NONE;
	}

	if (device_fd < 0) {
		LOGE("invalid fd %d", device_fd);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	FD_ZERO(&fds);
	FD_SET(device_fd, &fds);

	memset(&timeout, 0x0, sizeof(struct timeval));

	timeout.tv_sec = wait_time;
	timeout.tv_usec = 0;

	/*LOGD("select : %d sec", wait_time);*/

	ret = select(device_fd + 1, &fds, NULL, NULL, &timeout);
	if (ret == -1) {
		if (EINTR == errno) {
			LOGD("select error : EINTR");
			return CAMERA_ERROR_NONE;
		}
		LOGE("select failed. errno %d", errno);
		return CAMERA_ERROR_INTERNAL;
	}

	if (ret == 0) {
		LOGE("select timeout.");
		return CAMERA_ERROR_INTERNAL;
	}

	/*LOGD("select done");*/

	return CAMERA_ERROR_NONE;
}


static int __camera_v4l2_g_ctrl(int device_fd, int cid, int *value)
{
	int ret = 0;
	struct v4l2_control ctrl;

	if (!value) {
		LOGE("NULL param");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	memset(&ctrl, 0x0, sizeof(struct v4l2_control));

	ctrl.id = cid;

	if (!g_is_virtual_camera_mode)
		ret = v4l2_ioctl(device_fd, VIDIOC_G_CTRL, &ctrl);

	if (ret == 0)
		*value = ctrl.value;

	LOGD("G_CTRL id 0x%x, value %d, ret %d", cid, *value, ret);

	return ret;
}


static int __camera_v4l2_s_ctrl(int device_fd, int cid, int value)
{
	int ret = 0;
	struct v4l2_control ctrl;

	memset(&ctrl, 0x0, sizeof(struct v4l2_control));

	ctrl.id = cid;
	ctrl.value = value;

	if (!g_is_virtual_camera_mode)
		ret = v4l2_ioctl(device_fd, VIDIOC_S_CTRL, &ctrl);

	LOGD("S_CTRL id 0x%x, value %d, ret %d", cid, value, ret);

	return ret;
}


static int __camera_v4l2_stream(int device_fd, int type, gboolean onoff)
{
	if (device_fd < 0) {
		LOGE("invalid fd %d", device_fd);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(device_fd, onoff ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type) < 0) {
			LOGE("stream %d failed. [t:%d] errno %d", onoff, type, errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	LOGD("stream %d done [t:%d]", onoff, type);

	return CAMERA_ERROR_NONE;
}


static int __camera_v4l2_reqbufs(int device_fd, int type, int memory, uint32_t count, uint32_t *result_count)
{
	struct v4l2_requestbuffers v4l2_reqbuf;

	if (device_fd < 0) {
		LOGE("invalid fd %d", device_fd);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (!result_count) {
		LOGE("NULL parameter");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("type[%d], memory[%d], count[%u]", type, memory, count);

	memset(&v4l2_reqbuf, 0x0, sizeof(struct v4l2_requestbuffers));

	v4l2_reqbuf.type = type;
	v4l2_reqbuf.memory = memory;
	v4l2_reqbuf.count = count;

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(device_fd, VIDIOC_REQBUFS, &v4l2_reqbuf) < 0) {
			LOGE("REQBUFS[count %d] failed. errno %d", count, errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	if (v4l2_reqbuf.count != count)
		LOGW("different count [req:%d, result:%d]", count, v4l2_reqbuf.count);

	*result_count = v4l2_reqbuf.count;

	return CAMERA_ERROR_NONE;
}


static int __camera_v4l2_qbuf(int device_fd, int type, int memory, int index, int dmabuf_fd)
{
	struct v4l2_buffer v4l2_buf;
	struct v4l2_plane v4l2_planes[V4L2_PLANES_MAX];

	if (device_fd < 0) {
		LOGE("invalid fd %d", device_fd);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	memset(&v4l2_buf, 0x0, sizeof(struct v4l2_buffer));
	memset(v4l2_planes, 0x0, sizeof(v4l2_planes));

	v4l2_buf.index = index;
	v4l2_buf.type = type;
	v4l2_buf.memory = memory;
	v4l2_buf.m.planes = v4l2_planes;

	if (dmabuf_fd > CAMERA_HAL_INITIAL_FD) {
		if (V4L2_TYPE_IS_MULTIPLANAR(type))
			v4l2_buf.m.planes[0].m.fd = dmabuf_fd;
		else
			v4l2_buf.m.fd = dmabuf_fd;
	}

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(device_fd, VIDIOC_QBUF, &v4l2_buf) < 0) {
			LOGE("qbuf failed.  [i: %d, t: %d, m: %d] errno %d",
				index, type, memory, errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	LOGD("QBUF done [i:%d, t:%d, m:%d, fd:%d]",
		index, type, memory, dmabuf_fd);

	return CAMERA_ERROR_NONE;
}


static int __camera_v4l2_dqbuf(int device_fd, int type, int memory, int *index, uint32_t *bytesused)
{
	int ret = CAMERA_ERROR_NONE;
	static int virtual_buffer_index = 0;
	struct v4l2_buffer v4l2_buf;
	struct v4l2_plane v4l2_planes[V4L2_PLANES_MAX];

	if (device_fd < 0) {
		LOGE("invalid fd %d", device_fd);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (!index) {
		LOGE("NULL parameter");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	memset(&v4l2_buf, 0x0, sizeof(struct v4l2_buffer));
	memset(v4l2_planes, 0x0, sizeof(v4l2_planes));

	v4l2_buf.type = type;
	v4l2_buf.memory = memory;
	v4l2_buf.m.planes = v4l2_planes;

	if (!g_is_virtual_camera_mode) {
		ret = v4l2_ioctl(device_fd, VIDIOC_DQBUF, &v4l2_buf);
		if (ret < 0) {
			LOGE("dqbuf failed. [t: %d, m: %d] errno %d",
				type, memory, errno);
			return CAMERA_ERROR_DEVICE_READ;
		}
	} else {
		v4l2_buf.index = virtual_buffer_index++;
		v4l2_buf.bytesused = g_virtual_camera_bytesused;

		if (virtual_buffer_index >= BUFFER_MAX)
			virtual_buffer_index = 0;
	}

	LOGD("DQBUF[i:%d], bytesused[%u], length[%u]",
		v4l2_buf.index, v4l2_buf.bytesused, v4l2_buf.length);

	*index = v4l2_buf.index;
	if (bytesused)
		*bytesused = v4l2_buf.bytesused;

	/*LOGD("dqbuf index %d", *index);*/

	return CAMERA_ERROR_NONE;
}


static int __camera_get_format(guint32 fourcc, int *pixel_format)
{
	if (!pixel_format) {
		LOGE("NULL parameter");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	switch (fourcc) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12MT:
		*pixel_format = CAMERA_PIXEL_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		*pixel_format = CAMERA_PIXEL_FORMAT_NV21;
		break;
	case V4L2_PIX_FMT_YUV420:
		*pixel_format = CAMERA_PIXEL_FORMAT_I420;
		break;
	case V4L2_PIX_FMT_YVU420:
		*pixel_format = CAMERA_PIXEL_FORMAT_YV12;
		break;
	case V4L2_PIX_FMT_YUYV:
		*pixel_format = CAMERA_PIXEL_FORMAT_YUYV;
		break;
	case V4L2_PIX_FMT_UYVY:
		*pixel_format = CAMERA_PIXEL_FORMAT_UYVY;
		break;
	case V4L2_PIX_FMT_JPEG:
		*pixel_format = CAMERA_PIXEL_FORMAT_JPEG;
		break;
	case V4L2_PIX_FMT_H264:
		*pixel_format = CAMERA_PIXEL_FORMAT_H264;
		break;
	case V4L2_PIX_FMT_MJPEG:
		*pixel_format = CAMERA_PIXEL_FORMAT_MJPEG;
		break;
	default:
		LOGE("unknown fourcc "FOURCC_FORMAT, FOURCC_CONVERT(fourcc));
		return CAMERA_ERROR_INTERNAL;
	}

	LOGD("fourcc "FOURCC_FORMAT" -> %d",
		FOURCC_CONVERT(fourcc), *pixel_format);

	return CAMERA_ERROR_NONE;
}


static int __camera_get_fourcc_plane_num(int pixel_format, guint32 *fourcc, guint32 *plane_num)
{
	if (!fourcc || !plane_num) {
		LOGE("NULL parameter %p %p", fourcc, plane_num);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	switch (pixel_format) {
	case CAMERA_PIXEL_FORMAT_NV12:
		*fourcc = V4L2_PIX_FMT_NV12;
		*plane_num = 2;
		break;
	case CAMERA_PIXEL_FORMAT_NV21:
		*fourcc = V4L2_PIX_FMT_NV21;
		*plane_num = 2;
		break;
	case CAMERA_PIXEL_FORMAT_I420:
		*fourcc = V4L2_PIX_FMT_YUV420;
		*plane_num = 3;
		break;
	case CAMERA_PIXEL_FORMAT_YV12:
		*fourcc = V4L2_PIX_FMT_YVU420;
		*plane_num = 3;
		break;
	case CAMERA_PIXEL_FORMAT_YUYV:
		*fourcc = V4L2_PIX_FMT_YUYV;
		*plane_num = 1;
		break;
	case CAMERA_PIXEL_FORMAT_UYVY:
		*fourcc = V4L2_PIX_FMT_UYVY;
		*plane_num = 1;
		break;
	case CAMERA_PIXEL_FORMAT_JPEG:
		*fourcc = V4L2_PIX_FMT_JPEG;
		*plane_num = 1;
		break;
	case CAMERA_PIXEL_FORMAT_H264:
		*fourcc = V4L2_PIX_FMT_H264;
		*plane_num = 1;
		break;
	case CAMERA_PIXEL_FORMAT_MJPEG:
		*fourcc = V4L2_PIX_FMT_MJPEG;
		*plane_num = 1;
		break;
	default:
		LOGE("unknown format %d", pixel_format);
		return CAMERA_ERROR_INTERNAL;
	}

	LOGD("format %d -> fourcc "FOURCC_FORMAT,
		pixel_format, FOURCC_CONVERT(*fourcc));

	return CAMERA_ERROR_NONE;
}


static void __camera_get_fps_list(int device_fd, guint32 pixel_format, int width, int height, camera_fps_list_s *fps_list)
{
	uint32_t fps_count = 0;
	struct v4l2_frmivalenum ival;

	if (device_fd < 0 || !fps_list) {
		LOGE("invalid param %d %p", device_fd, fps_list);
		return;
	}

	if (g_is_virtual_camera_mode) {
		fps_list->fps[0] = 30;
		fps_list->count = 1;
		g_virtual_camera_frame_delay = G_USEC_PER_SEC / fps_list->fps[0];
		return;
	}

	ival.index = 0;
	ival.pixel_format = pixel_format;
	ival.width = width;
	ival.height = height;

	while (v4l2_ioctl(device_fd, VIDIOC_ENUM_FRAMEINTERVALS, &ival) >= 0) {
		if (ival.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
			LOGE("NOT DISCRETE type[%u] for [%dx%d]", ival.type, width, height);
			return;
		}

		if (ival.index++ >= FPS_COUNT_MAX) {
			LOGW("\t\t\t\tFramerate[i:%u][%u/%u] is available, but list is full[max:%d]",
				ival.index - 1, ival.discrete.denominator, ival.discrete.numerator, FPS_COUNT_MAX);
			continue;
		}

		LOGI("\t\t\t\tFramerate[%u/%u]", ival.discrete.denominator, ival.discrete.numerator);
		fps_list->fps[fps_count++] = ival.discrete.denominator;
	}


	fps_list->count = fps_count;
}


static int __camera_get_device_info(int device_index, int device_fd, camera_device_info_s *device_info, char *node_path)
{
	int format_index = 0;
	int format_count = 0;
	int resolution_index = 0;
	int resolution_count = 0;
	int camera_format = 0;
	struct v4l2_fmtdesc v4l2_format;
	struct v4l2_frmsizeenum v4l2_frame;

	if (device_fd < 0 || !device_info || !node_path) {
		LOGE("invalid param %d %p %p", device_fd, device_info, node_path);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGD("Get Supported format, resolution and fps");

	for (format_index = 0, format_count = 0 ; ; format_index++) {
		memset(&v4l2_format, 0x0, sizeof(struct v4l2_fmtdesc));

		v4l2_format.index = format_index;
		v4l2_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (!g_is_virtual_camera_mode) {
			if (v4l2_ioctl(device_fd, VIDIOC_ENUM_FMT, &v4l2_format) < 0) {
				LOGW("\tformat : end of enumeration");
				break;
			}
		} else {
			if (format_count == VIRTUAL_CAMERA_FMT_MAX)
				break;

			v4l2_format.pixelformat = g_virtual_camera_format[format_count];
		}

		LOGD("\tTry [%d] format "FOURCC_FORMAT" (emulated:%d)",
			format_count, FOURCC_CONVERT(v4l2_format.pixelformat),
			((v4l2_format.flags & V4L2_FMT_FLAG_EMULATED) ? 1 : 0));

		if (__camera_get_format(v4l2_format.pixelformat, &camera_format) != CAMERA_ERROR_NONE)
			continue;

		if (format_count + 1 >= CAMERA_PIXEL_FORMAT_MAX) {
			LOGW("format list is full[max:%u], skip format[i:%u][%d]",
				CAMERA_PIXEL_FORMAT_MAX, v4l2_format.index, camera_format);
			continue;
		}

		device_info->format_list.formats[format_count++] = camera_format;

		for (resolution_index = 0, resolution_count = 0 ; ; resolution_index++) {
			memset(&v4l2_frame, 0x0, sizeof(struct v4l2_frmsizeenum));

			v4l2_frame.index = resolution_index;
			v4l2_frame.pixel_format = v4l2_format.pixelformat;

			if (!g_is_virtual_camera_mode) {
				if (v4l2_ioctl(device_fd, VIDIOC_ENUM_FRAMESIZES, &v4l2_frame) < 0) {
					LOGW("\t\tframesize : end of enumeration");
					break;
				}
			} else {
				if (resolution_count == VIRTUAL_CAMERA_RES_MAX)
					break;

				v4l2_frame.discrete.width = g_virtual_camera_res[resolution_count].width;
				v4l2_frame.discrete.height = g_virtual_camera_res[resolution_count].height;
				v4l2_frame.type = V4L2_FRMSIZE_TYPE_DISCRETE;
			}

			if (resolution_count + 1 >= RESOLUTION_COUNT_MAX) {
				LOGW("resolution list is full, skip resolution[%ux%u]", v4l2_frame.discrete.width, v4l2_frame.discrete.height);
				continue;
			}

			switch (v4l2_frame.type) {
			case V4L2_FRMSIZE_TYPE_DISCRETE:
				device_info->preview_list.resolutions[resolution_count].width = v4l2_frame.discrete.width;
				device_info->preview_list.resolutions[resolution_count].height = v4l2_frame.discrete.height;
				device_info->capture_list.resolutions[resolution_count].width = v4l2_frame.discrete.width;
				device_info->capture_list.resolutions[resolution_count].height = v4l2_frame.discrete.height;
				device_info->video_list.resolutions[resolution_count].width = v4l2_frame.discrete.width;
				device_info->video_list.resolutions[resolution_count].height = v4l2_frame.discrete.height;

				LOGD("\t\tsize[%d] %ux%u", resolution_count,
					v4l2_frame.discrete.width,
					v4l2_frame.discrete.height);

				__camera_get_fps_list(device_fd,
					v4l2_frame.pixel_format,
					v4l2_frame.discrete.width,
					v4l2_frame.discrete.height,
					&device_info->preview_fps_list[resolution_count]);

				memcpy(&device_info->video_fps_list[resolution_count], &device_info->preview_fps_list[resolution_count], sizeof(camera_fps_list_s));

				resolution_count++;
				break;
			case V4L2_FRMSIZE_TYPE_CONTINUOUS:
				LOGW("\t\tsize[%d] %ux%u - %ux%u", resolution_count,
					v4l2_frame.stepwise.min_width,
					v4l2_frame.stepwise.min_height,
					v4l2_frame.stepwise.max_width,
					v4l2_frame.stepwise.max_height);
				break;
			case V4L2_FRMSIZE_TYPE_STEPWISE:
				LOGW("\t\tsize[%d] %ux%u - %ux%u (step %ux%u)", resolution_count,
					v4l2_frame.stepwise.min_width,
					v4l2_frame.stepwise.min_height,
					v4l2_frame.stepwise.max_width,
					v4l2_frame.stepwise.max_height,
					v4l2_frame.stepwise.step_width,
					v4l2_frame.stepwise.step_height);
				break;
			default:
				LOGE("\t\tunknown frame type %d", v4l2_frame.type);
				break;
			}
		}

		device_info->preview_list.count = resolution_count;
		device_info->capture_list.count = resolution_count;
		device_info->video_list.count = resolution_count;

		LOGD("\t\tresolution count [%d]", resolution_count);
	}

	device_info->index = device_index;
	device_info->format_list.count = format_count;
	device_info->facing_direction = CAMERA_FACING_DIRECTION_EXTERNAL;
	snprintf(device_info->name, sizeof(device_info->name), "V4L2_CAMERA");
	snprintf(device_info->node_path, sizeof(device_info->node_path), "%s", node_path);

	LOGD("\tformat count [%d]", format_count);

	return CAMERA_ERROR_NONE;
}


static int __camera_get_device_info_list(void)
{
	int i = 0;
	int ret = CAMERA_ERROR_NONE;
	int device_count = 0;
	int device_fd = CAMERA_HAL_INITIAL_FD;
#ifdef HAVE_LIBV4L2
	int libv4l2_fd = CAMERA_HAL_INITIAL_FD;
#endif /* HAVE_LIBV4L2 */
	glob_t glob_buf;
	struct v4l2_capability v4l2_cap;
	guint32 device_caps;
	camera_device_info_list_s *device_info_list = NULL;

	g_mutex_lock(&g_device_info_lock);

	if (g_device_info_list) {
		LOGD("device info list is already existed");
		ret = CAMERA_ERROR_NONE;
		goto _GET_DEVICE_INFO_LIST_DONE;
	}

	device_info_list = g_new0(camera_device_info_list_s, 1);
	if (!device_info_list) {
		LOGE("failed to alloc device info structure");
		ret = CAMERA_ERROR_OUT_OF_MEMORY;
		goto _GET_DEVICE_INFO_LIST_DONE;
	}

	if (g_is_virtual_camera_mode) {
		device_count = 1;

		ret = __camera_get_device_info(0, 0, &device_info_list->device_info[0], "VIRTUAL_CAMERA");

		device_info_list->count = device_count;
		g_device_info_list = device_info_list;

		goto _GET_DEVICE_INFO_LIST_DONE;
	}

	memset(&glob_buf, 0x0, sizeof(glob_t));

	ret = glob(DEVICE_NODE_PATH_PREFIX"*", 0, 0, &glob_buf);
	if (ret != 0) {
		switch (ret) {
		case GLOB_NOSPACE:
			LOGE("out of memory");
			ret = CAMERA_ERROR_OUT_OF_MEMORY;
			goto _GET_DEVICE_INFO_LIST_DONE;
		case GLOB_ABORTED:
			LOGE("read error");
			ret = CAMERA_ERROR_INTERNAL;
			goto _GET_DEVICE_INFO_LIST_DONE;
		case GLOB_NOMATCH:
			LOGE("match not found");
			ret = CAMERA_ERROR_INTERNAL;
			goto _GET_DEVICE_INFO_LIST_DONE;
		default:
			LOGE("unknown error : %d", ret);
			ret = CAMERA_ERROR_INTERNAL;
			goto _GET_DEVICE_INFO_LIST_DONE;
		}
	}

	LOGD("device node count : %zu", glob_buf.gl_pathc);

	for (i = 0 ; i < glob_buf.gl_pathc ; i++) {
		LOGD("[%d] check device [%s]", i, glob_buf.gl_pathv[i]);

		device_fd = open(glob_buf.gl_pathv[i], O_RDWR);
		if (device_fd < 0) {
			LOGE("open failed [%s] errno %d", glob_buf.gl_pathv[i], errno);
			continue;
		}

#ifdef HAVE_LIBV4L2
		libv4l2_fd = v4l2_fd_open(device_fd, V4L2_ENABLE_ENUM_FMT_EMULATION);

		LOGI("device_fd[%d], libv4l2_fd[%d]", device_fd, libv4l2_fd);

		if (libv4l2_fd != CAMERA_HAL_INITIAL_FD)
			device_fd = libv4l2_fd;
#endif /* HAVE_LIBV4L2 */

		memset(&v4l2_cap, 0x0, sizeof(struct v4l2_capability));

		if (v4l2_ioctl(device_fd, VIDIOC_QUERYCAP, &v4l2_cap) < 0) {
			LOGE("querycap failed. errno %d", errno);
			v4l2_close(device_fd);
			continue;
		}

		if (v4l2_cap.capabilities & V4L2_CAP_DEVICE_CAPS)
			device_caps = v4l2_cap.device_caps;
		else
			device_caps = v4l2_cap.capabilities;

		if (!(device_caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) ||
			(device_caps & (V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_VIDEO_OUTPUT_MPLANE))) {
			LOGW("[%s] is not a capture device 0x%x", glob_buf.gl_pathv[i], device_caps);
			v4l2_close(device_fd);
			continue;
		}

		ret = __camera_get_device_info(device_count, device_fd,
			&device_info_list->device_info[device_count], glob_buf.gl_pathv[i]);

		v4l2_close(device_fd);

		if (ret == CAMERA_ERROR_NONE) {
			device_count++;
			g_device_caps = device_caps;
			LOGI("device caps [0x%08x]", g_device_caps);
		}
	}

	device_info_list->count = device_count;
	g_device_info_list = device_info_list;

	LOGD("new g_device_info_list %p - device count %d",
		g_device_info_list, device_count);

_GET_DEVICE_INFO_LIST_DONE:
	g_mutex_unlock(&g_device_info_lock);
	LOGD("ret 0x%x", ret);

	if (ret != CAMERA_ERROR_NONE)
		g_free(device_info_list);

	return ret;
}


static int __camera_stop_stream(hal_camera_handle *handle, uint32_t buffer_count)
{
	int i = 0;
	int ret = CAMERA_ERROR_NONE;
	camera_buffer_s *buffer = NULL;
#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
	camera_tbm_buffer_s *tbm_buffer = NULL;
#endif

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("buffer count[%d]", buffer_count);

	/* stream off */
	ret = __camera_v4l2_stream(handle->device_fd, handle->buffer_type, FALSE);

	LOGI("stream off[0x%x]", ret);

	/* release buffer */
	for (i = 0 ; i < buffer_count ; i++) {
		buffer = &handle->camera_buffers[i];

		LOGI("release buffer[%d]", i);

#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
		tbm_buffer = &handle->tbm_buffers[i];

		if (tbm_buffer->dmabuf_fd > CAMERA_HAL_INITIAL_FD) {
			LOGI("  close dmabuf fd[%d]", tbm_buffer->dmabuf_fd);
			close(tbm_buffer->dmabuf_fd);
			tbm_buffer->dmabuf_fd = CAMERA_HAL_INITIAL_FD;
		}

		if (tbm_buffer->bo) {
			LOGI("  unref tbm bo[%p]", tbm_buffer->bo);
			tbm_bo_unref(tbm_buffer->bo);
			tbm_buffer->bo = NULL;
		}

		tbm_buffer->bo_handle.ptr = NULL;
#endif

		if (buffer->planes[0].data) {
#ifndef TIZEN_FEATURE_ZERO_COPY_SUPPORT
			LOGI("  munmap %p", buffer->planes[0].data);
			v4l2_munmap(buffer->planes[0].data, buffer->planes[0].size);
#endif
			buffer->planes[0].data = NULL;
			buffer->planes[0].size = 0;
		} else {
			LOGI("  NULL data [i:%d]", i);
		}
	}

	ret = __camera_v4l2_reqbufs(handle->device_fd,
		handle->buffer_type, handle->memory_type, 0, &buffer_count);

	LOGI("reqbufs 0 [0x%x]", ret);

	return ret;
}


static int __camera_start_stream(hal_camera_handle *handle, camera_pixel_format_e pixel_format,
	camera_resolution_s *resolution, uint32_t fps, uint32_t request_buffer_count)
{
	int i = 0;
	int ret = CAMERA_ERROR_NONE;
	camera_buffer_s *buffer = NULL;
	struct v4l2_format v4l2_fmt;
	struct v4l2_streamparm v4l2_parm;
#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
	uint32_t buffer_size = 0;
	camera_tbm_buffer_s *tbm_buffer = NULL;
#else
	struct v4l2_buffer v4l2_buf;
	struct v4l2_plane v4l2_planes[V4L2_PLANES_MAX];;
#endif
	guint32 fourcc = 0;
	guint32 plane_num = 0;

	if (!handle || !resolution) {
		LOGE("NULL param %p %p", handle, resolution);
		return CAMERA_ERROR_INTERNAL;
	}

	LOGI("[%dx%d], format[%d], fps[%u]",
		resolution->width, resolution->height, pixel_format, fps);

	/* S_FMT */
	ret = __camera_get_fourcc_plane_num(pixel_format, &fourcc, &plane_num);
	if (ret != CAMERA_ERROR_NONE) {
		LOGE("get fourcc failed [format %d]", pixel_format);
		return ret;
	}

	memset(&v4l2_fmt, 0x0, sizeof(struct v4l2_format));

	v4l2_fmt.type = handle->buffer_type;
	if (V4L2_TYPE_IS_MULTIPLANAR(handle->buffer_type)) {
		v4l2_fmt.fmt.pix_mp.width = resolution->width;
		v4l2_fmt.fmt.pix_mp.height = resolution->height;
		v4l2_fmt.fmt.pix_mp.pixelformat = fourcc;
		v4l2_fmt.fmt.pix_mp.num_planes = plane_num;
	} else {
		v4l2_fmt.fmt.pix.width = resolution->width;
		v4l2_fmt.fmt.pix.height = resolution->height;
		v4l2_fmt.fmt.pix.pixelformat = fourcc;
		v4l2_fmt.fmt.pix.bytesperline = resolution->width;
	}

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(handle->device_fd, VIDIOC_S_FMT, &v4l2_fmt) < 0) {
			LOGE("S_FMT failed. errno %d", errno);
			return CAMERA_ERROR_INTERNAL;
		}
	} else {
		tbm_surface_h surface = NULL;
		tbm_surface_info_s sinfo;

		surface = tbm_surface_create(resolution->width, resolution->height, TBM_FORMAT_YUV420);
		tbm_surface_get_info(surface, &sinfo);
		tbm_surface_destroy(surface);

		g_virtual_camera_bytesused = v4l2_fmt.fmt.pix.sizeimage = sinfo.size;
	}

	if (V4L2_TYPE_IS_MULTIPLANAR(handle->buffer_type)) {
		for (i = 0 ; i < v4l2_fmt.fmt.pix_mp.num_planes ; i++) {
#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
			/* TODO: Is it needed to make array for each planes? */
			buffer_size += v4l2_fmt.fmt.pix_mp.plane_fmt[i].sizeimage;
#endif
			LOGD("plane[%d] stride %u, sizeimage %u", i,
				v4l2_fmt.fmt.pix_mp.plane_fmt[i].bytesperline,
				v4l2_fmt.fmt.pix_mp.plane_fmt[i].sizeimage);
		}
	} else {
#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
		buffer_size = v4l2_fmt.fmt.pix.sizeimage;
#endif
		LOGD("stride %d, sizeimage %d",
			v4l2_fmt.fmt.pix.bytesperline,
			v4l2_fmt.fmt.pix.sizeimage);
	}

	/* G_PARM */
	memset(&v4l2_parm, 0x0, sizeof(struct v4l2_streamparm));

	v4l2_parm.type = handle->buffer_type;

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(handle->device_fd, VIDIOC_G_PARM, &v4l2_parm) < 0) {
			LOGE("G_PARM failed. errno %d", errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	/* S_PARM to set fps */
	v4l2_parm.parm.capture.timeperframe.numerator = 1;
	v4l2_parm.parm.capture.timeperframe.denominator = fps;

	if (!g_is_virtual_camera_mode) {
		if (v4l2_ioctl(handle->device_fd, VIDIOC_S_PARM, &v4l2_parm) < 0) {
			LOGE("S_PARM failed. errno %d", errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	/* request buffer */
	ret = __camera_v4l2_reqbufs(handle->device_fd,
		handle->buffer_type, handle->memory_type,
		request_buffer_count, &handle->buffer_count);
	if (ret != CAMERA_ERROR_NONE) {
		return ret;
	}

	LOGI("buffer count : request %d -> result %d",
		request_buffer_count, handle->buffer_count);

	/* alloc and queue buffer */
	for (i = 0 ; i < handle->buffer_count ; i++) {
		buffer = &handle->camera_buffers[i];

		buffer->index = i;
		buffer->format = pixel_format;
		buffer->resolution.width = resolution->width;
		buffer->resolution.height = resolution->height;
		buffer->num_planes = plane_num;

#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
		tbm_buffer = &handle->tbm_buffers[i];

		tbm_buffer->bo = tbm_bo_alloc(handle->bufmgr, (int)buffer_size, TBM_BO_DEFAULT);
		if (!tbm_buffer->bo) {
			LOGE("bo alloc failed[size:%u]", buffer_size);
			ret = CAMERA_ERROR_OUT_OF_MEMORY;
			goto _START_STREAM_FAILED;
		}

		tbm_buffer->bo_handle = tbm_bo_get_handle(tbm_buffer->bo, TBM_DEVICE_CPU);
		if (!tbm_buffer->bo_handle.ptr) {
			LOGE("bo[%p] get handle failed", tbm_buffer->bo);
			ret = CAMERA_ERROR_INTERNAL;
			goto _START_STREAM_FAILED;
		}

		tbm_buffer->dmabuf_fd = tbm_bo_export_fd(tbm_buffer->bo);
		if (tbm_buffer->dmabuf_fd < 0) {
			LOGE("export fd failed from bo[%p]", tbm_buffer->bo);
			ret = CAMERA_ERROR_INTERNAL;
			goto _START_STREAM_FAILED;
		}

		buffer->total_size = buffer_size;
		buffer->planes[0].size = buffer_size;
		buffer->planes[0].data = tbm_buffer->bo_handle.ptr;
		buffer->num_bos = 1;
		buffer->bos[0] = tbm_buffer->bo;

		LOGD("buffer[%d], size[%d], data[%p], bo[%p], fd[%d]",
			i, buffer->planes[0].size, buffer->planes[0].data,
			tbm_buffer->bo, tbm_buffer->dmabuf_fd);

		ret = __camera_v4l2_qbuf(handle->device_fd,
			handle->buffer_type, handle->memory_type,
			i, tbm_buffer->dmabuf_fd);
#else /* TIZEN_FEATURE_ZERO_COPY_SUPPORT */
		memset(&v4l2_buf, 0x0, sizeof(struct v4l2_buffer));
		memset(v4l2_planes, 0x0, sizeof(v4l2_planes));

		v4l2_buf.type = handle->buffer_type;
		v4l2_buf.memory = handle->memory_type;
		v4l2_buf.index = i;
		v4l2_buf.m.planes = v4l2_planes;
		v4l2_buf.length = plane_num;

		if (v4l2_ioctl(handle->device_fd, VIDIOC_QUERYBUF, &v4l2_buf) < 0) {
			LOGE("[%d] query buf failed. errno %d", i, errno);
			ret = CAMERA_ERROR_INTERNAL;
			goto _START_STREAM_FAILED;
		}

		buffer->index = i;
		buffer->format = pixel_format;
		buffer->resolution.width = resolution->width;
		buffer->resolution.height = resolution->height;
		buffer->total_size = v4l2_buf.length;
		buffer->num_planes = plane_num;
		buffer->planes[0].size = v4l2_buf.length;
		buffer->planes[0].data = v4l2_mmap(0,
			v4l2_buf.length,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			handle->device_fd,
			v4l2_buf.m.offset);

		if (buffer->planes[0].data == MAP_FAILED) {
			LOGE("[%d] mmap failed (errno %d)", i, errno);
			goto _START_STREAM_FAILED;
		}

		LOGI("buffer[%d], size[%d], data[%p]",
			i, buffer->planes[0].size, buffer->planes[0].data);

		ret = __camera_v4l2_qbuf(handle->device_fd,
			handle->buffer_type, handle->memory_type,
			i, CAMERA_HAL_INITIAL_FD);
#endif /* TIZEN_FEATURE_ZERO_COPY_SUPPORT */

		if (ret != CAMERA_ERROR_NONE) {
			LOGE("[%d] qbuf failed (errno %d)", i, errno);
			goto _START_STREAM_FAILED;
		}
	}

	/* stream on */
	ret = __camera_v4l2_stream(handle->device_fd, handle->buffer_type, TRUE);
	if (ret != CAMERA_ERROR_NONE) {
		LOGE("stream on failed");
		goto _START_STREAM_FAILED;
	}

	LOGI("done");

	return CAMERA_ERROR_NONE;

_START_STREAM_FAILED:
	__camera_stop_stream(handle, handle->buffer_count);
	return ret;
}


static void __camera_do_capture(hal_camera_handle *handle)
{
	int ret = CAMERA_ERROR_NONE;
	int index = 0;
	gint64 current_time = 0;
	gint64 previous_time = 0;
	gint64 interval_us = 0;
	uint32_t bytesused = 0;

	if (!handle) {
		LOGE("NULL handle");
		return;
	}

	LOGD("start");

	if (handle->capture_count > 1)
		interval_us = (gint64)handle->capture_interval_ms * 1000;

	/* restart stream for capture */
	if (handle->capture_restart_stream) {
		ret = __camera_stop_stream(handle, handle->buffer_count);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("stop stream failed for capture[0x%x]", ret);
			goto _CAPTURE_DONE;
		}

		ret = __camera_start_stream(handle,
			handle->preview_format.capture_format,
			&handle->preview_format.capture_resolution,
			handle->preview_format.stream_fps,
			BUFFER_MAX);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("start stream failed for capture[0x%x]", ret);
			goto _CAPTURE_DONE;
		}
	}

	do {
		/* get capture buffer */
		ret = __camera_v4l2_wait_frame(handle->device_fd, 5);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("frame wait failed for capture[0x%x]", ret);
			goto _CAPTURE_DONE;
		}

		ret = __camera_v4l2_dqbuf(handle->device_fd,
			handle->buffer_type, handle->memory_type,
			&index, &bytesused);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("dqbuf failed for capture[0x%x]", ret);
			goto _CAPTURE_DONE;
		}

		handle->camera_buffers[index].planes[0].bytesused = bytesused;

		if (handle->captured_count > 0) {
			g_mutex_lock(&handle->buffer_lock);
			if (handle->state != CAMERA_STATE_CAPTURING) {
				LOGW("stop continuous capture");
				handle->captured_count = handle->capture_count;
				g_mutex_unlock(&handle->buffer_lock);
				goto _TRY_NEXT;
			}
			g_mutex_unlock(&handle->buffer_lock);
		}

		if (handle->capture_count > 1) {
			current_time = g_get_monotonic_time();

			LOGI("time[prev:%"PRId64", cur:%"PRId64"] interval[%"PRId64" us]",
				previous_time, current_time, interval_us);

			if (current_time < previous_time + interval_us)
				goto _TRY_NEXT;
		}

		g_mutex_lock(&handle->buffer_lock);
		handle->captured_count++;
		g_mutex_unlock(&handle->buffer_lock);

		LOGD("capture cb[%p], buffer index[%d],count[%d]",
			handle->capture_cb, index, handle->captured_count);

		if (handle->capture_cb) {
			handle->capture_cb(&handle->camera_buffers[index],
				NULL, NULL, handle->capture_cb_data);
		} else {
			LOGW("capture callback is NULL");
			/* Need to post error? */
		}

		previous_time = current_time;

_TRY_NEXT:
		ret = __camera_v4l2_qbuf(handle->device_fd,
			handle->buffer_type, handle->memory_type,
			index, handle->tbm_buffers[index].dmabuf_fd);
		if (ret != CAMERA_ERROR_NONE)
			LOGE("qbuf failed for capture[0x%x]", ret);
	} while (handle->captured_count < handle->capture_count);

	g_mutex_lock(&handle->buffer_lock);

	if (handle->state == CAMERA_STATE_CAPTURING) {
		LOGD("wait for capture stop signal");
		g_cond_wait(&handle->buffer_cond, &handle->buffer_lock);
		LOGD("signal received");
	} else {
		LOGD("The state is already changed.");
	}

	g_mutex_unlock(&handle->buffer_lock);

_CAPTURE_DONE:
	/* restart stream for preview */
	if (handle->capture_restart_stream) {
		ret = __camera_stop_stream(handle, handle->buffer_count);
		if (ret != CAMERA_ERROR_NONE)
			LOGE("stop stream failed for preview[0x%x]", ret);

		ret = __camera_start_stream(handle,
			handle->preview_format.stream_format,
			&handle->preview_format.stream_resolution,
			handle->preview_format.stream_fps,
			BUFFER_MAX);
		if (ret != CAMERA_ERROR_NONE)
			LOGE("start stream failed for preview[0x%x]", ret);
	}

	LOGD("done");
}


static void *__camera_buffer_handler_func(gpointer data)
{
	int error = CAMERA_ERROR_NONE;
	int index = 0;
	hal_camera_handle *handle = (hal_camera_handle *)data;
	uint32_t bytesused = 0;

	if (!handle) {
		LOGE("NULL handle for buffer handler");
		return NULL;
	}

	LOGD("enter");

	/* run buffer thread */
	g_mutex_lock(&handle->buffer_lock);

	while (handle->buffer_thread_run) {
		g_mutex_unlock(&handle->buffer_lock);

		if (__camera_v4l2_wait_frame(handle->device_fd, 5) != CAMERA_ERROR_NONE) {
			LOGE("frame wait failed");
			g_mutex_lock(&handle->buffer_lock);
			break;
		}

		g_mutex_lock(&handle->buffer_lock);

		if (handle->buffer_thread_run == FALSE) {
			LOGW("stop buffer handler thread");
			break;
		}

		error = __camera_v4l2_dqbuf(handle->device_fd,
			handle->buffer_type, handle->memory_type,
			&index, &bytesused);
		if (error != CAMERA_ERROR_NONE) {
			LOGE("dqbuf failed[0x%x]", error);
			break;
		}

		handle->buffer_dequeued_count++;
		handle->camera_buffers[index].planes[0].bytesused = bytesused;

		/*LOGD("dequeued buffer count %d", handle->buffer_dequeued_count);*/

		g_mutex_unlock(&handle->buffer_lock);

		if (handle->preview_cb) {
			handle->preview_cb(&handle->camera_buffers[index], NULL, handle->preview_cb_data);
		} else {
			LOGW("preview callback is NULL");
			camera_v4l2_release_preview_buffer((void *)handle, index);
		}

		g_mutex_lock(&handle->extra_preview_lock);

		if (handle->extra_preview_cb) {
			handle->extra_preview_cb(&handle->camera_buffers[index], NULL, 0, handle->extra_preview_cb_data);
			handle->extra_preview_cb(&handle->camera_buffers[index], NULL, 1, handle->extra_preview_cb_data);
			handle->extra_preview_cb(&handle->camera_buffers[index], NULL, 2, handle->extra_preview_cb_data);
			handle->extra_preview_cb(&handle->camera_buffers[index], NULL, 3, handle->extra_preview_cb_data);
		}

		g_mutex_unlock(&handle->extra_preview_lock);

		/* check capture request flag */
		if (handle->capture_request) {
			__camera_do_capture(handle);
			handle->capture_request = FALSE;
		}

		sched_yield();

		g_mutex_lock(&handle->buffer_lock);
	}

	g_mutex_unlock(&handle->buffer_lock);

	if (error != CAMERA_ERROR_NONE)
		__camera_send_message(handle, CAMERA_MESSAGE_TYPE_ERROR, error);

	LOGD("leave");

	return NULL;
}


static void __camera_message_release_func(gpointer data)
{
	camera_message_s *message = (camera_message_s *)data;

	if (!message) {
		LOGW("NULL message");
		return;
	}

	LOGD("release message %p, type %d", message, message->type);

	g_free(message);

	return;
}


static void *_camera_message_handler_func(gpointer data)
{
	int i = 0;
	camera_message_s *message = NULL;
	hal_camera_handle *handle = (hal_camera_handle *)data;

	if (!handle) {
		LOGE("NULL handle for capture thread");
		return NULL;
	}

	LOGD("enter - message thread");

	g_mutex_lock(&handle->msg_cb_lock);

	while (handle->msg_cb_run) {
		if (g_queue_is_empty(handle->msg_list)) {
			LOGD("wait for message");
			g_cond_wait(&handle->msg_cb_cond, &handle->msg_cb_lock);
			LOGD("message signal received");
		}

		if (!handle->msg_cb_run) {
			LOGW("break message thread");
			break;
		}

		message = g_queue_pop_head(handle->msg_list);
		if (!message) {
			LOGW("NULL message");
			continue;
		}

		g_mutex_unlock(&handle->msg_cb_lock);

		for (i = 0 ; i < MESSAGE_CALLBACK_MAX ; i++) {
			if (handle->msg_cb[i]) {
				LOGD("call message callback[%d] type[%d]", i, message->type);
				handle->msg_cb[i](message, handle->msg_cb_data[i]);
			}
		}

		g_free(message);
		message = NULL;

		g_mutex_lock(&handle->msg_cb_lock);
	}

	g_mutex_unlock(&handle->msg_cb_lock);

	LOGD("leave - message thread");

	return NULL;
}


static void __camera_release_handle(hal_camera_handle *handle)
{
	if (!handle) {
		LOGW("NULL handle");
		return;
	}

	if (handle->msg_thread) {
		g_mutex_lock(&handle->msg_cb_lock);
		handle->msg_cb_run = FALSE;
		g_cond_signal(&handle->msg_cb_cond);
		g_mutex_unlock(&handle->msg_cb_lock);
		g_thread_join(handle->msg_thread);
		g_queue_free_full(handle->msg_list, (GDestroyNotify)__camera_message_release_func);
		handle->msg_list = NULL;
	}

	g_mutex_clear(&handle->lock);
	g_mutex_clear(&handle->buffer_lock);
	g_mutex_clear(&handle->msg_cb_lock);
	g_mutex_clear(&handle->extra_preview_lock);
	g_cond_clear(&handle->buffer_cond);
	g_cond_clear(&handle->msg_cb_cond);

	if (handle->bufmgr) {
		tbm_bufmgr_deinit(handle->bufmgr);
		handle->bufmgr = NULL;
	}

	LOGD("camera HAL handle %p destroy", handle);

	g_free(handle);

	return;
}


int camera_v4l2_init(void **camera_handle)
{
	int i = 0;
	int ret = CAMERA_ERROR_NONE;
	hal_camera_handle *new_handle = NULL;
	tbm_bufmgr bufmgr = NULL;

	LOGD("enter");

	if (!camera_handle) {
		LOGE("NULL pointer for handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	ret = __camera_get_device_info_list();
	if (ret != CAMERA_ERROR_NONE) {
		LOGE("get device info failed[0x%x]", ret);
		return ret;
	}

	bufmgr = tbm_bufmgr_init(-1);
	if (bufmgr == NULL) {
		LOGE("get tbm bufmgr failed");
		return CAMERA_ERROR_INTERNAL;
	}

	new_handle = g_new0(hal_camera_handle, 1);
	if (!new_handle) {
		LOGE("failed to alloc camera hal handle");
		tbm_bufmgr_deinit(bufmgr);
		return CAMERA_ERROR_OUT_OF_MEMORY;
	}

	new_handle->bufmgr = bufmgr;

	g_mutex_init(&new_handle->lock);
	g_mutex_init(&new_handle->buffer_lock);
	g_mutex_init(&new_handle->msg_cb_lock);
	g_mutex_init(&new_handle->extra_preview_lock);
	g_cond_init(&new_handle->buffer_cond);
	g_cond_init(&new_handle->msg_cb_cond);

	/* message thread */
	new_handle->msg_list = g_queue_new();
	new_handle->msg_cb_run = TRUE;
	new_handle->msg_thread = g_thread_try_new("camera_hal_msg_thread",
		_camera_message_handler_func, (gpointer)new_handle, NULL);
	if (!new_handle->msg_thread) {
		LOGE("failed to create message thread");
		ret = CAMERA_ERROR_INTERNAL;
		goto _INIT_ERROR;
	}

	new_handle->device_index = CAMERA_HAL_INITIAL_INDEX;
	new_handle->device_fd = CAMERA_HAL_INITIAL_FD;
	new_handle->state = CAMERA_STATE_INITIALIZED;

	for (i = 0 ; i < BUFFER_MAX ; i++)
		new_handle->tbm_buffers[i].dmabuf_fd = CAMERA_HAL_INITIAL_FD;

#ifdef HAVE_LIBV4L2
	LOGI("libv4l2 ENABLED");
#else /* HAVE_LIBV4L2 */
	LOGI("libv4l2 DISABLED");
#endif /* HAVE_LIBV4L2 */

	*camera_handle = new_handle;

	LOGD("camera HAL handle %p", new_handle);

	return CAMERA_ERROR_NONE;

_INIT_ERROR:
	__camera_release_handle(new_handle);

	return ret;
}


int camera_v4l2_deinit(void *camera_handle)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_INITIALIZED) {
		LOGE("invalid state %d, can not destroy handle", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	g_mutex_unlock(&handle->lock);

	__camera_release_handle(handle);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_get_device_info_list(camera_device_info_list_s *device_info_list)
{
	int ret = 0;

	if (!device_info_list) {
		LOGE("NULL pointer for device_info_list");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	ret = __camera_get_device_info_list();
	if (ret != CAMERA_ERROR_NONE) {
		LOGE("get device info failed");
		return ret;
	}

	memcpy(device_info_list, g_device_info_list, sizeof(camera_device_info_list_s));

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_open_device(void *camera_handle, int device_index)
{
	int ret = CAMERA_ERROR_NONE;
	int device_fd = CAMERA_HAL_INITIAL_FD;
#ifdef HAVE_LIBV4L2
	int libv4l2_fd = CAMERA_HAL_INITIAL_FD;
#endif /* HAVE_LIBV4L2 */
	char *node_path = NULL;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_INITIALIZED) {
		LOGE("invalid state %d", handle->state);
		ret = CAMERA_ERROR_INVALID_STATE;
		goto _OPEN_DEVICE_EXIT;
	}

	if (!g_device_info_list) {
		LOGE("NO DEVICE INFO");
		ret = CAMERA_ERROR_INTERNAL;
		goto _OPEN_DEVICE_EXIT;
	}

	if (device_index >= g_device_info_list->count) {
		LOGE("invalid index %d [info:%d]", device_index, g_device_info_list->count);
		ret = CAMERA_ERROR_INVALID_PARAMETER;
		goto _OPEN_DEVICE_EXIT;
	}

	node_path = g_device_info_list->device_info[device_index].node_path;

	if (g_is_virtual_camera_mode) {
		device_fd = 0;
		handle->buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		handle->memory_type = V4L2_MEMORY_DMABUF;
		goto _OPEN_DEVICE_DONE;
	}

	device_fd = open(node_path, O_RDWR);
	if (device_fd < 0) {
		switch (errno) {
			case EACCES:
			case EPERM:
				ret = CAMERA_ERROR_PERMISSION_DENIED;
				break;
			case ENOENT:
				ret = CAMERA_ERROR_DEVICE_NOT_FOUND;
				break;
			case EBUSY:
				ret = CAMERA_ERROR_DEVICE_BUSY;
				break;
			default:
				ret = CAMERA_ERROR_DEVICE_OPEN;
				break;
		}

		LOGE("open [%s] failed 0x%x [errno %d]",
			node_path, ret, errno);

		goto _OPEN_DEVICE_EXIT;
	}

	if (g_device_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		handle->buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else
		handle->buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

#ifdef TIZEN_FEATURE_ZERO_COPY_SUPPORT
	handle->memory_type = V4L2_MEMORY_DMABUF;
#else
	handle->memory_type = V4L2_MEMORY_MMAP;
#endif

#ifdef HAVE_LIBV4L2
	libv4l2_fd = v4l2_fd_open(device_fd, V4L2_ENABLE_ENUM_FMT_EMULATION);

	LOGI("device_fd[%d], libv4l2_fd[%d]", device_fd, libv4l2_fd);

	if (libv4l2_fd != CAMERA_HAL_INITIAL_FD)
		device_fd = libv4l2_fd;
#endif /* HAVE_LIBV4L2 */

_OPEN_DEVICE_DONE:
	handle->state = CAMERA_STATE_OPENED;
	handle->device_index = device_index;
	handle->device_fd = device_fd;

	LOGD("[%d] device[%s] opened [fd %d, type %d]",
		device_index, node_path, device_fd, handle->buffer_type);

_OPEN_DEVICE_EXIT:
	g_mutex_unlock(&handle->lock);

	return ret;
}


int camera_v4l2_open_device_ext(void *camera_handle, const char *device_name)
{
	LOGE("NOT SUPPORTED");
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_close_device(void *camera_handle)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_OPENED) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	if (!g_is_virtual_camera_mode) {
		if (handle->device_fd >= 0) {
			LOGD("close fd %d", handle->device_fd);
			v4l2_close(handle->device_fd);
			handle->device_fd = CAMERA_HAL_INITIAL_FD;
		} else {
			LOGW("invalid fd %d", handle->device_fd);
		}
	}

	handle->state = CAMERA_STATE_INITIALIZED;

	LOGD("device [%d] closed", handle->device_index);

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_add_message_callback(void *camera_handle, hal_camera_message_cb callback, void *user_data, uint32_t *cb_id)
{
	uint32_t i = 0;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (!callback || !cb_id) {
		LOGE("NULL pointer for callback %p or cb_id %p", callback, cb_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	for (i = 0 ; i < MESSAGE_CALLBACK_MAX ; i++) {
		if (handle->msg_cb[i] == NULL) {
			handle->msg_cb[i] = callback;
			handle->msg_cb_data[i] = user_data;
			*cb_id = i;
			LOGD("message cb [%p] added, user data %p - id %u", callback, user_data, i);
			g_mutex_unlock(&handle->lock);
			return CAMERA_ERROR_NONE;
		}
	}

	g_mutex_unlock(&handle->lock);

	LOGE("no available message cb slot");

	return CAMERA_ERROR_INTERNAL;
}


int camera_v4l2_remove_message_callback(void *camera_handle, uint32_t cb_id)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (cb_id >= MESSAGE_CALLBACK_MAX) {
		LOGE("invalid cb_id %u", cb_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->msg_cb[cb_id]) {
		LOGD("remove message callback %p, user data %p - cb_id %u",
			handle->msg_cb[cb_id], handle->msg_cb_data[cb_id], cb_id);

		handle->msg_cb[cb_id] = NULL;
		handle->msg_cb_data[cb_id] = NULL;
	} else {
		LOGE("already removed message cb");
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INTERNAL;
	}

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_preview_stream_format(void *camera_handle, camera_format_s *format)
{
	int i = 0;
	int j = 0;
	int ret = CAMERA_ERROR_NONE;
	gboolean capability_check = FALSE;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;
	camera_device_info_s *device_info = NULL;

	if (!handle || !format) {
		LOGE("NULL param %p %p", handle, format);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (!g_device_info_list) {
		LOGE("no device info list");
		return CAMERA_ERROR_INTERNAL;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_OPENED &&
		handle->state != CAMERA_STATE_PREVIEWING) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	/* check capability */
	device_info = &g_device_info_list->device_info[handle->device_index];

	/* format */
	for (i = 0 ; i < device_info->format_list.count ; i++) {
		if (format->stream_format == device_info->format_list.formats[i]) {
			LOGD("format matched %d, check resolution.", format->stream_format);

			/* resolution */
			for (j = 0 ; j < device_info->preview_list.count ; j++) {
				if (format->stream_resolution.width == device_info->preview_list.resolutions[j].width &&
					format->stream_resolution.height == device_info->preview_list.resolutions[j].height) {
					LOGD("resolution matched %dx%d",
						format->stream_resolution.width,
						format->stream_resolution.height);
					capability_check = TRUE;
					break;
				}
			}

			break;
		}
	}

	if (!capability_check) {
		LOGE("capability failed - %d, %dx%d",
			format->stream_format,
			format->stream_resolution.width,
			format->stream_resolution.height);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	/* compare with current settings */
	if (handle->state == CAMERA_STATE_PREVIEWING) {
		if (handle->preview_format.stream_format == format->stream_format &&
			handle->preview_format.stream_resolution.width == format->stream_resolution.width &&
			handle->preview_format.stream_resolution.height == format->stream_resolution.height &&
			handle->preview_format.stream_fps == format->stream_fps &&
			handle->preview_format.stream_rotation == format->stream_rotation) {
			LOGD("no need to restart preview stream");
			goto _SET_PREVIEW_STREAM_FORMAT_DONE;
		}

		LOGD("Preview setting is changed. Restart preview now.");

		/* stop preview stream to change it */
		ret = __camera_stop_stream(handle, handle->buffer_count);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("failed to stop stream");
			g_mutex_unlock(&handle->lock);
			return ret;
		}

		/* restart preview stream to change it */
		ret = __camera_start_stream(handle,
			format->stream_format,
			&format->stream_resolution,
			format->stream_fps,
			BUFFER_MAX);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("failed to start stream");
			g_mutex_unlock(&handle->lock);
			return ret;
		}
	}

_SET_PREVIEW_STREAM_FORMAT_DONE:
	/* set capture restart flag */
	if (format->stream_format == format->capture_format &&
		format->stream_resolution.width == format->capture_resolution.width &&
		format->stream_resolution.height == format->capture_resolution.height)
		handle->capture_restart_stream = FALSE;
	else
		handle->capture_restart_stream = TRUE;

	memcpy(&handle->preview_format, format, sizeof(camera_format_s));

	LOGD("set format PREVIEW[%d:%dx%d,fps:%d], CAPTURE[%d:%dx%d,restart:%d]",
		format->stream_format,
		format->stream_resolution.width,
		format->stream_resolution.height,
		format->stream_fps,
		format->capture_format,
		format->capture_resolution.width,
		format->capture_resolution.height,
		handle->capture_restart_stream);

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_get_preview_stream_format(void *camera_handle, camera_format_s *format)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !format) {
		LOGE("NULL param %p %p", handle, format);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	memcpy(format, &handle->preview_format, sizeof(camera_format_s));

	LOGD("get stream format %d, %dx%d", format->stream_format,
		format->stream_resolution.width, format->stream_resolution.height);

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_user_buffer_fd(void *camera_handle, int *fds, int number)
{
	LOGE("NOT SUPPORTED");
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_start_preview(void *camera_handle, hal_camera_preview_frame_cb callback, void *user_data)
{
	int ret = 0;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !callback) {
		LOGE("NULL param %p %p", handle, callback);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_OPENED) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	ret = __camera_start_stream(handle,
		handle->preview_format.stream_format,
		&handle->preview_format.stream_resolution,
		handle->preview_format.stream_fps,
		BUFFER_MAX);
	if (ret != CAMERA_ERROR_NONE) {
		LOGE("__camera_start_stream failed[0x%x]", ret);
		g_mutex_unlock(&handle->lock);
		return ret;
	}

	g_mutex_lock(&handle->buffer_lock);

	handle->buffer_thread_run = TRUE;

	handle->buffer_thread = g_thread_try_new("camera_hal_buffer_thread",
		__camera_buffer_handler_func, (gpointer)handle, NULL);
	if (!handle->buffer_thread) {
		LOGE("failed to create buffer handler thread");
		g_mutex_unlock(&handle->buffer_lock);

		__camera_stop_stream(handle, handle->buffer_count);

		g_mutex_unlock(&handle->lock);

		return ret;
	}

	handle->preview_cb = callback;
	handle->preview_cb_data = user_data;

	g_mutex_unlock(&handle->buffer_lock);

	handle->state = CAMERA_STATE_PREVIEWING;

	LOGD("start preview done");

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_release_preview_buffer(void *camera_handle, int index)
{
	int ret = CAMERA_ERROR_NONE;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (index >= handle->buffer_count) {
		LOGE("invalid buffer index %d", index);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	ret = __camera_v4l2_qbuf(handle->device_fd,
		handle->buffer_type, handle->memory_type,
		index, handle->tbm_buffers[index].dmabuf_fd);

	g_mutex_lock(&handle->buffer_lock);

	if (ret == CAMERA_ERROR_NONE) {
		if (handle->buffer_dequeued_count > 0)
			handle->buffer_dequeued_count--;
		else
			LOGW("invalid dequeued buffer count[%u]", handle->buffer_dequeued_count);

		/*LOGD("qbud done : index %d, dequeued buffer count %d",
			index, handle->buffer_dequeued_count);*/
	} else {
		LOGE("qbuf failed [index %d]", index);
	}

	g_cond_signal(&handle->buffer_cond);

	g_mutex_unlock(&handle->buffer_lock);

	return ret;
}


int camera_v4l2_stop_preview(void *camera_handle)
{
	int ret = CAMERA_ERROR_NONE;
	gint64 end_time;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGD("start");

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_PREVIEWING) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	g_mutex_lock(&handle->buffer_lock);

	handle->buffer_thread_run = FALSE;

	while (handle->buffer_dequeued_count > 0) {
		LOGD("wait for dequeued buffer [%d]", handle->buffer_dequeued_count);
		end_time = g_get_monotonic_time() + 3 * G_TIME_SPAN_SECOND;
		if (!g_cond_wait_until(&handle->buffer_cond, &handle->buffer_lock, end_time)) {
			LOGE("buffer wait failed");
			break;
		} else {
			LOGD("signal received. check again...");
		}
	}

	g_mutex_unlock(&handle->buffer_lock);

	/* wait for preview thread exit */
	LOGD("wait for buffer thread");

	g_thread_join(handle->buffer_thread);
	handle->buffer_thread = NULL;

	ret = __camera_stop_stream(handle, handle->buffer_count);

	handle->state = CAMERA_STATE_OPENED;

	LOGD("stop preview done [0x%x]", ret);

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_start_auto_focus(void *camera_handle)
{
	if (!camera_handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* auto focus is not supported */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_stop_auto_focus(void *camera_handle)
{
	if (!camera_handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* auto focus is not supported */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_start_capture(void *camera_handle, hal_camera_capture_cb callback, void *user_data)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !callback) {
		LOGE("NULL param %p %p", camera_handle, callback);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_PREVIEWING) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	/* set callback and user data */
	handle->capture_cb = callback;
	handle->capture_cb_data = user_data;

	/* reset captured count */
	handle->captured_count = 0;

	LOGD("start capture - count %u", handle->capture_count);

	/* set capture request flag */
	handle->capture_request = TRUE;

	handle->state = CAMERA_STATE_CAPTURING;

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_stop_capture(void *camera_handle)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	if (handle->state != CAMERA_STATE_CAPTURING) {
		LOGE("invalid state %d", handle->state);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INVALID_STATE;
	}

	g_mutex_lock(&handle->buffer_lock);

	if (handle->captured_count == 0) {
		LOGE("No captured image yet.");
		g_mutex_unlock(&handle->buffer_lock);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_INTERNAL;
	}

	LOGD("send signal to start preview after capture");

	g_cond_signal(&handle->buffer_cond);
	g_mutex_unlock(&handle->buffer_lock);

	handle->state = CAMERA_STATE_PREVIEWING;

	g_mutex_unlock(&handle->lock);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_video_stream_format(void *camera_handle, camera_format_s *format)
{
	if (!camera_handle || !format) {
		LOGE("NULL param %p %p", camera_handle, format);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* single stream device can not support video stream */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_get_video_stream_format(void *camera_handle, camera_format_s *format)
{
	if (!camera_handle || !format) {
		LOGE("NULL param %p %p", camera_handle, format);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* single stream device can not support video stream */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_start_record(void *camera_handle, hal_camera_video_frame_cb callback, void *user_data)
{
	if (!camera_handle || !callback) {
		LOGE("NULL param %p %p", camera_handle, callback);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* single stream device can not support video stream */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_release_video_buffer(void *camera_handle, int index)
{
	if (!camera_handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* single stream device can not support video stream */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_stop_record(void *camera_handle)
{
	if (!camera_handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGE("NOT SUPPORTED");

	/* single stream device can not support video stream */
	return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
}


int camera_v4l2_set_extra_preview_frame_cb(void *camera_handle, hal_camera_extra_preview_frame_cb callback, void *user_data)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->extra_preview_lock);

	handle->extra_preview_cb = callback;
	handle->extra_preview_cb_data = user_data;

	g_mutex_unlock(&handle->extra_preview_lock);

	LOGI("done");

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_unset_extra_preview_frame_cb(void *camera_handle)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->extra_preview_lock);

	handle->extra_preview_cb = NULL;
	handle->extra_preview_cb_data = NULL;

	g_mutex_unlock(&handle->extra_preview_lock);

	LOGI("done");

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_release_extra_preview_buffer(void *camera_handle, int stream_id, int index)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("done - stream_id[%d], index[%d]", stream_id, index);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_extra_preview_stream_format(void *camera_handle, int stream_id, camera_format_s *format)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("stream_id[%d], [%d,%dx%d,%d]",
		stream_id, format->stream_format,
		format->stream_resolution.width, format->stream_resolution.height,
		format->stream_fps);

	memcpy(&handle->extra_preview_format[stream_id], format, sizeof(camera_format_s));

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_get_extra_preview_stream_format(void *camera_handle, int stream_id, camera_format_s *format)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !format) {
		LOGE("NULL param[%p,%p]", handle, format);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	memcpy(format, &handle->extra_preview_format[stream_id], sizeof(camera_format_s));

	LOGI("stream_id[%d], [%d,%dx%d,%d]",
		stream_id, format->stream_format,
		format->stream_resolution.width, format->stream_resolution.height,
		format->stream_fps);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_extra_preview_bitrate(void *camera_handle, int stream_id, int bitrate)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("set bitrate[%d] for stream_id[%d]", bitrate, stream_id);

	handle->extra_preview_format[stream_id].stream_bitrate = (uint32_t)bitrate;

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_get_extra_preview_bitrate(void *camera_handle, int stream_id, int *bitrate)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !bitrate) {
		LOGE("NULL param[%p,%p]", handle, bitrate);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	*bitrate = (int)handle->extra_preview_format[stream_id].stream_bitrate;

	LOGI("get bitrate[%d] for stream_id[%d]", *bitrate, stream_id);

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_extra_preview_gop_interval(void *camera_handle, int stream_id, int interval)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle) {
		LOGE("NULL handle");
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	LOGI("set GOP interval[%d] for stream_id[%d]", interval, stream_id);

	handle->extra_preview_gop_interval[stream_id] = interval;

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_get_extra_preview_gop_interval(void *camera_handle, int stream_id, int *interval)
{
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !interval) {
		LOGE("NULL param[%p,%p]", handle, interval);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	if (stream_id < 0 || stream_id >= EXTRA_PREVIEW_STREAM_MAX) {
		LOGE("invalid stream_id[%d]", stream_id);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	*interval = (int)handle->extra_preview_gop_interval[stream_id];

	LOGI("get GOP interval[%d] for stream_id[%d]", *interval, stream_id);

	return CAMERA_ERROR_NONE;
}


static int __set_command(hal_camera_handle *handle, int64_t command, void *value)
{
	int cid = 0;
	int ctrl_ret = 0;
	int set_value = 0;

	if (!handle || !value) {
		LOGE("NULL param %p %p", handle, value);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	set_value = *(int *)value;

	if (handle->state < CAMERA_STATE_OPENED) {
		LOGE("invalid state %d", handle->state);
		return CAMERA_ERROR_INVALID_STATE;
	}

	LOGI("command[%"PRIx64"] -> [%d]", command, set_value);

	switch (command) {
	case CAMERA_COMMAND_BRIGHTNESS:
		cid = V4L2_CID_BRIGHTNESS;
		break;
	case CAMERA_COMMAND_CONTRAST:
		cid = V4L2_CID_CONTRAST;
		break;
	case CAMERA_COMMAND_SATURATION:
		cid = V4L2_CID_SATURATION;
		break;
	case CAMERA_COMMAND_SHARPNESS:
		cid = V4L2_CID_SHARPNESS;
		break;
	case CAMERA_COMMAND_PTZ_TYPE:
		if (set_value != CAMERA_PTZ_TYPE_ELECTRONIC) {
			LOGE("not supported PTZ type %d", set_value);
			return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
		}
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_PAN:
		cid = V4L2_CID_PAN_ABSOLUTE;
		break;
	case CAMERA_COMMAND_TILT:
		cid = V4L2_CID_TILT_ABSOLUTE;
		break;
	case CAMERA_COMMAND_EXPOSURE:
		cid = V4L2_CID_EXPOSURE;
		break;
	case CAMERA_COMMAND_FLIP:
		if (set_value != CAMERA_FLIP_NONE) {
			LOGE("NOT_SUPPORTED flip %d", set_value);
			return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
		}
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_CAPTURE_COUNT:
		handle->capture_count = set_value;
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_CAPTURE_INTERVAL:
		handle->capture_interval_ms = set_value;
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_FOCUS_MODE:
		handle->focus_mode = set_value;
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_FOCUS_RANGE:
		handle->focus_range = set_value;
		return CAMERA_ERROR_NONE;
	case CAMERA_COMMAND_FOCUS_LEVEL:
		handle->focus_level = set_value;
		return CAMERA_ERROR_NONE;
	default:
		LOGE("NOT_SUPPORTED command %"PRIx64, command);
		return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
	}

	ctrl_ret = __camera_v4l2_s_ctrl(handle->device_fd, cid, set_value);
	if (ctrl_ret < 0) {
		switch (errno) {
		case EACCES:
		case EPERM:
			LOGE("Permission denied %d", errno);
			return CAMERA_ERROR_PERMISSION_DENIED;
		case EINVAL:
			LOGE("Invalid argument");
			return CAMERA_ERROR_INVALID_PARAMETER;
		case EBUSY:
			LOGE("Device busy");
			return CAMERA_ERROR_DEVICE_BUSY;
		case ENOTSUP:
			LOGE("Not supported");
			return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
		default:
			LOGE("Unknown errro %d", errno);
			return CAMERA_ERROR_INTERNAL;
		}
	}

	return CAMERA_ERROR_NONE;
}


int camera_v4l2_set_command(void *camera_handle, int64_t command, void *value)
{
	int ret = CAMERA_ERROR_NONE;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	g_mutex_lock(&handle->lock);

	ret = __set_command(handle, command, value);

	g_mutex_unlock(&handle->lock);

	return ret;
}


int camera_v4l2_get_command(void *camera_handle, int64_t command, void **value)
{
	int ret = CAMERA_ERROR_NONE;
	int cid = 0;
	int ctrl_ret = 0;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;

	if (!handle || !value) {
		LOGE("NULL param %p %p", handle, value);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	LOGI("command[%"PRIx64"]", command);

	switch (command) {
	case CAMERA_COMMAND_BRIGHTNESS:
		cid = V4L2_CID_BRIGHTNESS;
		break;
	case CAMERA_COMMAND_CONTRAST:
		cid = V4L2_CID_CONTRAST;
		break;
	case CAMERA_COMMAND_SATURATION:
		cid = V4L2_CID_SATURATION;
		break;
	case CAMERA_COMMAND_SHARPNESS:
		cid = V4L2_CID_SHARPNESS;
		break;
	case CAMERA_COMMAND_EXPOSURE:
		cid = V4L2_CID_EXPOSURE;
		break;
	case CAMERA_COMMAND_FOCUS_MODE:
		**(int **)value = handle->focus_mode;
		goto _GET_COMMAND_DONE;
	case CAMERA_COMMAND_FOCUS_RANGE:
		**(int **)value = handle->focus_range;
		goto _GET_COMMAND_DONE;
	case CAMERA_COMMAND_FOCUS_LEVEL:
		**(int **)value = handle->focus_level;
		goto _GET_COMMAND_DONE;
	default:
		LOGE("Not supported command[%"PRIx64"]", command);
		g_mutex_unlock(&handle->lock);
		return CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
	}

	ctrl_ret = __camera_v4l2_g_ctrl(handle->device_fd, cid, (int *)*value);
	if (ctrl_ret < 0) {
		switch (errno) {
		case EACCES:
		case EPERM:
			LOGE("Permission denied %d", errno);
			ret = CAMERA_ERROR_PERMISSION_DENIED;
			break;
		case EINVAL:
			LOGE("Invalid argument");
			ret = CAMERA_ERROR_INVALID_PARAMETER;
			break;
		case EBUSY:
			LOGE("Device busy");
			ret = CAMERA_ERROR_DEVICE_BUSY;
			break;
		case ENOTSUP:
			LOGE("Not supported");
			ret = CAMERA_ERROR_DEVICE_NOT_SUPPORTED;
			break;
		default:
			LOGE("Unknown errro %d", errno);
			ret = CAMERA_ERROR_INTERNAL;
			break;
		}
	}

_GET_COMMAND_DONE:
	if (ret == CAMERA_ERROR_NONE)
		LOGI("get[%d]", **(int **)value);

	g_mutex_unlock(&handle->lock);

	return ret;
}


static void __dump_batch_command(camera_batch_command_control_s *batch_command)
{
	if (!batch_command) {
		LOGE("NULL batch command");
		return;
	}

	LOGI("[WHITE_BALANCE] %d", batch_command->white_balance);
	LOGI("[ISO] %d", batch_command->iso);
	LOGI("[CONTRAST] %d", batch_command->contrast);
	LOGI("[HUE] %d", batch_command->hue);
	LOGI("[SATURATION] %d", batch_command->saturation);
	LOGI("[SHARPNESS] %d", batch_command->sharpness);
	LOGI("[BRIGHTNESS] %d", batch_command->brightness);
	LOGI("[EFFECT] %d", batch_command->effect);
	LOGI("[SCENE_MODE] %d", batch_command->scene_mode);
	LOGI("[EXPOSURE_MODE] %d", batch_command->exposure_mode);
	LOGI("[EXPOSURE] %d", batch_command->exposure);
	LOGI("[ROTATION] %d", batch_command->rotation);
	LOGI("[FLIP] %d", batch_command->flip);
	LOGI("[FOCUS_MODE] %d", batch_command->focus_mode);
	LOGI("[FOCUS_RANGE] %d", batch_command->focus_range);
	LOGI("[FOCUS_AREA] %d,%d,%dx%d", batch_command->focus_area.x, batch_command->focus_area.y,
		batch_command->focus_area.width, batch_command->focus_area.height);
	LOGI("[FOCUS_LEVEL] %d", batch_command->focus_level);
	LOGI("[SHOT_MODE] %d", batch_command->shot_mode);
	LOGI("[ANTI_SHAKE] %d", batch_command->anti_shake);
	LOGI("[DIGITAL_ZOOM] %d", batch_command->digital_zoom);
	LOGI("[OPTICAL_ZOOM] %d", batch_command->optical_zoom);
	LOGI("[RECORDING_HINT] %d", batch_command->recording_hint);
	LOGI("[WDR] %d", batch_command->wdr);
	LOGI("[SHUTTER_SPEED] %d/%d", batch_command->shutter_speed.numerator, batch_command->shutter_speed.denominator);
	LOGI("[FLASH_MODE] %d", batch_command->flash_mode);
	LOGI("[FLASH_BRIGHTNESS] %d", batch_command->flash_brightness);
	LOGI("[FACE_DETECTION] %d", batch_command->face_detection);
	LOGI("[PTZ_TYPE] %d", batch_command->ptz_type);
	LOGI("[PAN] %d", batch_command->pan);
	LOGI("[TILT] %d", batch_command->tilt);
	LOGI("[BITRATE] %d", batch_command->bitrate);
	LOGI("[GOP_INTERVAL] %d", batch_command->gop_interval);
	LOGI("[CAPTURE_COUNT] %d", batch_command->capture_count);
	LOGI("[CAPTURE_INTERVAL] %d", batch_command->capture_interval);
}


int camera_v4l2_set_batch_command(void *camera_handle, camera_batch_command_control_s *batch_command, int64_t *error_command)
{
	int ret = CAMERA_ERROR_NONE;
	int i = 0;
	int support_count = 0;
	hal_camera_handle *handle = (hal_camera_handle *)camera_handle;
	set_batch_table_s set_table[] = {
		{CAMERA_COMMAND_BRIGHTNESS, batch_command ? (void *)&batch_command->brightness : NULL},
		{CAMERA_COMMAND_CONTRAST, batch_command ? (void *)&batch_command->contrast : NULL},
		{CAMERA_COMMAND_SATURATION, batch_command ? (void *)&batch_command->saturation : NULL},
		{CAMERA_COMMAND_SHARPNESS, batch_command ? (void *)&batch_command->sharpness : NULL},
		{CAMERA_COMMAND_PTZ_TYPE, batch_command ? (void *)&batch_command->ptz_type : NULL},
		{CAMERA_COMMAND_PAN, batch_command ? (void *)&batch_command->pan : NULL},
		{CAMERA_COMMAND_TILT, batch_command ? (void *)&batch_command->tilt : NULL},
		{CAMERA_COMMAND_FLIP, batch_command ? (void *)&batch_command->flip : NULL},
		{CAMERA_COMMAND_CAPTURE_COUNT, batch_command ? (void *)&batch_command->capture_count : NULL},
		{CAMERA_COMMAND_CAPTURE_INTERVAL, batch_command ? (void *)&batch_command->capture_interval : NULL},
		{CAMERA_COMMAND_FOCUS_MODE, batch_command ? (void *)&batch_command->focus_mode : NULL},
		{CAMERA_COMMAND_FOCUS_RANGE, batch_command ? (void *)&batch_command->focus_range : NULL},
		{CAMERA_COMMAND_FOCUS_LEVEL, batch_command ? (void *)&batch_command->focus_level : NULL}
	};

	if (!handle || !batch_command) {
		LOGE("NULL param %p %p", handle, batch_command);
		return CAMERA_ERROR_INVALID_PARAMETER;
	}

	g_mutex_lock(&handle->lock);

	support_count = sizeof(set_table) / sizeof(set_batch_table_s);

	LOGD("set batch command - support count %d", support_count);

	__dump_batch_command(batch_command);

	for (i = 0 ; i < support_count ; i++) {
		if (!(batch_command->command_set_flag & set_table[i].command))
			continue;

		ret = __set_command(handle, set_table[i].command, set_table[i].value);
		if (ret != CAMERA_ERROR_NONE) {
			LOGE("failed command %"PRIx64", ret 0x%x", set_table[i].command, ret);
			break;
		}
	}

	g_mutex_unlock(&handle->lock);

	return ret;
}


static int camera_v4l2_backend_init(void **data)
{
	hal_backend_camera_funcs *funcs;

	funcs = calloc(1, sizeof(hal_backend_camera_funcs));
	if (!funcs)
		return CAMERA_ERROR_OUT_OF_MEMORY;

	funcs->init = camera_v4l2_init;
	funcs->deinit = camera_v4l2_deinit;
	funcs->get_device_info_list = camera_v4l2_get_device_info_list;
	funcs->open_device = camera_v4l2_open_device;
	funcs->open_device_ext = camera_v4l2_open_device_ext;
	funcs->close_device = camera_v4l2_close_device;
	funcs->add_message_callback = camera_v4l2_add_message_callback;
	funcs->remove_message_callback = camera_v4l2_remove_message_callback;
	funcs->set_preview_stream_format = camera_v4l2_set_preview_stream_format;
	funcs->get_preview_stream_format = camera_v4l2_get_preview_stream_format;
	funcs->set_user_buffer_fd = camera_v4l2_set_user_buffer_fd;
	funcs->start_preview = camera_v4l2_start_preview;
	funcs->release_preview_buffer = camera_v4l2_release_preview_buffer;
	funcs->stop_preview = camera_v4l2_stop_preview;
	funcs->start_auto_focus = camera_v4l2_start_auto_focus;
	funcs->stop_auto_focus = camera_v4l2_stop_auto_focus;
	funcs->start_capture = camera_v4l2_start_capture;
	funcs->stop_capture = camera_v4l2_stop_capture;
	funcs->set_video_stream_format = camera_v4l2_set_video_stream_format;
	funcs->get_video_stream_format = camera_v4l2_get_video_stream_format;
	funcs->start_record = camera_v4l2_start_record;
	funcs->release_video_buffer = camera_v4l2_release_video_buffer;
	funcs->stop_record = camera_v4l2_stop_record;
	funcs->set_command = camera_v4l2_set_command;
	funcs->get_command = camera_v4l2_get_command;
	funcs->set_batch_command = camera_v4l2_set_batch_command;
	funcs->set_extra_preview_frame_cb = camera_v4l2_set_extra_preview_frame_cb;
	funcs->unset_extra_preview_frame_cb = camera_v4l2_unset_extra_preview_frame_cb;
	funcs->release_extra_preview_buffer = camera_v4l2_release_extra_preview_buffer;
	funcs->set_extra_preview_stream_format = camera_v4l2_set_extra_preview_stream_format;
	funcs->get_extra_preview_stream_format = camera_v4l2_get_extra_preview_stream_format;
	funcs->set_extra_preview_bitrate = camera_v4l2_set_extra_preview_bitrate;
	funcs->get_extra_preview_bitrate = camera_v4l2_get_extra_preview_bitrate;
	funcs->set_extra_preview_gop_interval = camera_v4l2_set_extra_preview_gop_interval;
	funcs->get_extra_preview_gop_interval = camera_v4l2_get_extra_preview_gop_interval;

	*data = (void *)funcs;

	g_is_virtual_camera_mode = getenv(ENV_VIRTUAL_CAMERA) ? TRUE : FALSE;

	LOGW("VIRTUAL CAMERA mode[%d]", g_is_virtual_camera_mode);

	return 0;
}


static int camera_v4l2_backend_exit(void *data)
{
	if (!data)
		return 0;

	free(data);

	return 0;
}


hal_backend hal_backend_camera_data = {
	.name = "camera-v4l2",
	.vendor = "TIZEN",
	.abi_version = HAL_ABI_VERSION_TIZEN_6_5,
	.init = camera_v4l2_backend_init,
	.exit = camera_v4l2_backend_exit,
};
