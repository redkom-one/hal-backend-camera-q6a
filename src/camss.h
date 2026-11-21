/*
 * camss.h - Qualcomm camss (RDI raw path) + Bayer sensor support for the
 * Tizen V4L2 camera HAL on the Radxa Dragon Q6A.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef __HAL_BACKEND_CAMERA_CAMSS_H__
#define __HAL_BACKEND_CAMERA_CAMSS_H__

#include <stdint.h>
#include <glib.h>
#include <hal/hal-camera-interface.h>

typedef struct camss_ctx camss_ctx;

/* TRUE when a media device has a raw sensor wired to camss. */
gboolean camss_detect(void);

/* Fill the HAL device info for the camss camera. */
void camss_fill_device_info(camera_device_info_s *info, int index);

/* Set up the media pipeline and open the nodes. Returns a CAMERA_ERROR_* in *error on failure. */
camss_ctx *camss_open(int *error);
void camss_close(camss_ctx *ctx);

/* fd to select() on while streaming. */
int camss_get_fd(camss_ctx *ctx);

int camss_start(camss_ctx *ctx, uint32_t fps);
int camss_stop(camss_ctx *ctx);

/*
 * Dequeue one raw frame, convert it into out (out->format and out->resolution
 * pick the conversion; out->planes[0].data must hold camss_frame_size() bytes),
 * requeue the raw buffer and run auto exposure / white balance.
 * out == NULL drops the frame (statistics still run).
 */
int camss_get_frame(camss_ctx *ctx, camera_buffer_s *out);

/* Bytes needed for one output frame, 0 for an unsupported format. */
uint32_t camss_frame_size(camera_pixel_format_e format, uint32_t width, uint32_t height);

#endif /* __HAL_BACKEND_CAMERA_CAMSS_H__ */
