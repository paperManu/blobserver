pluginsdir = $(prefix)/lib/blobserver-@LIBBLOBSERVER_API_VERSION@

plugins_LTLIBRARIES = \
	libactuatorbgsubtractor.la \
	libactuatordepthtouch.la \
	libactuatorfiducialtracker.la \
	libactuatorhog.la \
	libactuatorlightspots.la \
	libactuatormeanoutliers.la \
	libactuatornop.la \
	libactuatorobjonaplane.la \
	libactuatorstitch.la

if PCL
plugins_LTLIBRARIES += \
    libactuatorarmpcl.la \
	libactuatorclusterpcl.la
endif

#-------------#
if PCL
libactuatorarmpcl_la_SOURCES = \
	armpcl.cpp

libactuatorarmpcl_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS) \
	$(PCL_CFLAGS)

libactuatorarmpcl_la_LIBADD = \
    $(PCL_LIBS)
endif

#-------------#
libactuatorbgsubtractor_la_SOURCES = \
	bgsubtractor.cpp

libactuatorbgsubtractor_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

#-------------#
if PCL
libactuatorclusterpcl_la_SOURCES = \
	clusterPcl.cpp

libactuatorclusterpcl_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS) \
	$(PCL_CFLAGS)

libactuatorclusterpcl_la_LDFLAGS = \
	$(PCL_LIBS)
endif

#-------------#
libactuatordepthtouch_la_SOURCES = \
	depthtouch.cpp

libactuatordepthtouch_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

#-------------#
libactuatorfiducialtracker_la_SOURCES = \
	fiducialtracker.cpp

libactuatorfiducialtracker_la_CXXFLAGS = \
	-I$(top_srcdir)/ext/libfidtrack \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

# External libs, compiled along blobserver
libactuatorfiducialtracker_la_SOURCES += \
    $(top_srcdir)/ext/libfidtrack/dump_graph.c \
    $(top_srcdir)/ext/libfidtrack/fidtrack120.c \
    $(top_srcdir)/ext/libfidtrack/fidtrackX.c \
    $(top_srcdir)/ext/libfidtrack/segment.c \
    $(top_srcdir)/ext/libfidtrack/threshold.c \
    $(top_srcdir)/ext/libfidtrack/tiled_bernsen_threshold.c \
    $(top_srcdir)/ext/libfidtrack/topologysearch.c \
    $(top_srcdir)/ext/libfidtrack/treeidmap.cpp

#-------------#
libactuatorhog_la_SOURCES = \
	hog.cpp \
	$(top_srcdir)/src/descriptor_hog.cpp

libactuatorhog_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

libactuatorhog_la_LDFLAGS = \
	-lopencv_ml

#-------------#
libactuatorlightspots_la_SOURCES = \
	lightSpots.cpp

libactuatorlightspots_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

libactuatorlightspots_la_LDFLAGS = \
	-lopencv_features2d

#-------------#
libactuatormeanoutliers_la_SOURCES = \
	meanOutliers.cpp

libactuatormeanoutliers_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

#-------------#
libactuatornop_la_SOURCES = \
	nop.cpp

libactuatornop_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

#-------------#
libactuatorobjonaplane_la_SOURCES = \
	objOnAPlane.cpp

libactuatorobjonaplane_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

#-------------#
libactuatorstitch_la_SOURCES = \
	stitch.cpp

libactuatorstitch_la_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

