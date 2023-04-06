/*
 * hal_backend_camera_v4l2_private.h
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

#ifndef __HAL_BACKEND_CAMERA_V4L2_PRIVATE_H__
#define __HAL_BACKEND_CAMERA_V4L2_PRIVATE_H__

#include <linux/videodev2.h>
#include <glib.h>
#include <tbm_bufmgr.h>
#include <hal/hal-common-interface.h>
#include <hal/hal-camera-interface.h>

#ifdef HAVE_LIBV4L2
#include <libv4l2.h>
#else
#define v4l2_fd_open(fd, flags) (fd)
#define v4l2_close              close
#define v4l2_dup                dup
#define v4l2_ioctl              ioctl
#define v4l2_read               read
#define v4l2_mmap               mmap
#define v4l2_munmap             munmap
#endif /* ENABLE_LIBV4L2 */

#define CAMERA_HAL_INITIAL_INDEX    -1
#define CAMERA_HAL_INITIAL_FD       -1
#define MESSAGE_CALLBACK_MAX        10
#define BUFFER_MAX                  4
#define V4L2_PLANES_MAX             4
#define EXTRA_PREVIEW_STREAM_MAX    10

typedef struct _set_batch_table_s {
	int64_t command;
	void *value;
} set_batch_table_s;

typedef struct _camera_tbm_buffer_s {
	tbm_bo bo;
	tbm_bo_handle bo_handle;
	int dmabuf_fd;
} camera_tbm_buffer_s;

typedef struct _camera_hal_handle {
	/* tbm */
	tbm_bufmgr bufmgr;

	/* device */
	gint32 device_index;
	gint32 device_fd;

	/* buffer */
	guint32 buffer_dequeued_count;
	GThread *buffer_thread;
	gboolean buffer_thread_run;
	guint32 buffer_count;
	camera_buffer_s camera_buffers[BUFFER_MAX];
	camera_tbm_buffer_s tbm_buffers[BUFFER_MAX];
	enum v4l2_buf_type buffer_type;
	enum v4l2_memory memory_type;
	GMutex buffer_lock;
	GCond buffer_cond;

	/* preview */
	camera_format_s preview_format;
	hal_camera_preview_frame_cb preview_cb;
	gpointer preview_cb_data;
	hal_camera_extra_preview_frame_cb extra_preview_cb;
	gpointer extra_preview_cb_data;
	GMutex extra_preview_lock;
	camera_format_s extra_preview_format[EXTRA_PREVIEW_STREAM_MAX];
	gint32 extra_preview_gop_interval[EXTRA_PREVIEW_STREAM_MAX];

	/* capture */
	hal_camera_capture_cb capture_cb;
	gpointer capture_cb_data;
	guint32 capture_count;
	guint32 capture_interval_ms;
	guint32 captured_count;
	gboolean capture_request;
	gboolean capture_restart_stream;

	/* message */
	GThread *msg_thread;
	hal_camera_message_cb msg_cb[MESSAGE_CALLBACK_MAX];
	gpointer msg_cb_data[MESSAGE_CALLBACK_MAX];
	gboolean msg_cb_run;
	GQueue *msg_list;
	GMutex msg_cb_lock;
	GCond msg_cb_cond;

	/* settings */
	gint32 focus_mode;

	/* etc */
	GMutex lock;
	camera_state_e state;
} hal_camera_handle;

int camera_v4l2_init(void **camera_handle);
int camera_v4l2_deinit(void *camera_handle);
int camera_v4l2_get_device_info_list(camera_device_info_list_s *device_info_list);
int camera_v4l2_open_device(void *camera_handle, int device_index);
int camera_v4l2_open_device_ext(void *camera_handle, const char *device_name);
int camera_v4l2_close_device(void *camera_handle);
int camera_v4l2_add_message_callback(void *camera_handle, hal_camera_message_cb callback, void *user_data, uint32_t *cb_id);
int camera_v4l2_remove_message_callback(void *camera_handle, uint32_t cb_id);
int camera_v4l2_set_preview_stream_format(void *camera_handle, camera_format_s *format);
int camera_v4l2_get_preview_stream_format(void *camera_handle, camera_format_s *format);
int camera_v4l2_set_user_buffer_fd(void *camera_handle, int *fds, int number);
int camera_v4l2_start_preview(void *camera_handle, hal_camera_preview_frame_cb callback, void *user_data);
int camera_v4l2_release_preview_buffer(void *camera_handle, int buffer_index);
int camera_v4l2_stop_preview(void *camera_handle);
int camera_v4l2_start_auto_focus(void *camera_handle);
int camera_v4l2_stop_auto_focus(void *camera_handle);
int camera_v4l2_start_capture(void *camera_handle, hal_camera_capture_cb callback, void *user_data);
int camera_v4l2_stop_capture(void *camera_handle);
int camera_v4l2_set_video_stream_format(void *camera_handle, camera_format_s *format);
int camera_v4l2_get_video_stream_format(void *camera_handle, camera_format_s *format);
int camera_v4l2_start_record(void *camera_handle, hal_camera_video_frame_cb callback, void *user_data);
int camera_v4l2_release_video_buffer(void *camera_handle, int buffer_index);
int camera_v4l2_stop_record(void *camera_handle);
int camera_v4l2_set_command(void *camera_handle, int64_t command, void *value);
int camera_v4l2_get_command(void *camera_handle, int64_t command, void **value);
int camera_v4l2_set_batch_command(void *camera_handle, camera_batch_command_control_s *batch_command, int64_t *error_command);

#endif /* __HAL_BACKEND_CAMERA_V4L2_PRIVATE_H__ */
