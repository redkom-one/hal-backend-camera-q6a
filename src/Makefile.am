ACLOCAL_AMFLAGS='-I m4'

lib_LTLIBRARIES = libhal-backend-camera.la

noinst_HEADERS = hal_backend_camera_v4l2_private.h

libhal_backend_camera_la_SOURCES = hal_backend_camera_v4l2.c

libhal_backend_camera_la_CFLAGS = \
	-I$(srcdir)/include \
	$(GLIB_CFLAGS) \
	$(DLOG_CFLAGS) \
	$(HAL_API_COMMON_CFLAGS) \
	$(HAL_API_CAMERA_CFLAGS) \
	$(TBM_CFLAGS)

libhal_backend_camera_la_LIBADD = \
	$(GLIB_LIBS) \
	$(DLOG_LIBS) \
	$(TBM_LIBS)

if HAVE_LIBV4L2
libhal_backend_camera_la_CFLAGS += $(LIBV4L2_CFLAGS) -DHAVE_LIBV4L2
libhal_backend_camera_la_LIBADD += $(LIBV4L2_LIBS)
endif

libhal_backend_camera_la_CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections
libhal_backend_camera_la_LDFLAGS = -Wl,--gc-sections -avoid-version

