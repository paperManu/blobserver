AM_CPPFLAGS = \
    -I$(top_srcdir)/include \
    -DG_LOG_DOMAIN=""

bin_PROGRAMS = \
    blobserver \
    blobcontroller \
    blobtrainer

# Blobserver
blobserver_SOURCES = \
    base_objects.cpp \
    blobserver.cpp \
    blob.cpp \
    blob_2D.cpp \
    blob_2D_color.cpp \
    configurator.cpp \
    descriptor_hog.cpp \
    actuator.cpp \
    actuator_armpcl.cpp \
    actuator_bgsubtractor.cpp \
    actuator_clusterPcl.cpp \
	actuator_depthtouch.cpp \
	actuator_fiducialtracker.cpp \
    actuator_hog.cpp \
    actuator_lightSpots.cpp \
    actuator_meanOutliers.cpp \
    actuator_objOnAPlane.cpp \
    actuator_stitch.cpp \
    actuator_nop.cpp \
	hdribuilder.cpp \
    source.cpp \
    source_2d.cpp \
    source_2d_opencv.cpp \
    source_2d_shmdata.cpp \
    source_3d_shmdata.cpp \
    threadPool.cpp

noinst_HEADERS = \
    $(top_srcdir)/include/abstract-factory.h \
    $(top_srcdir)/include/abstract-factory_spec.h \
    $(top_srcdir)/include/base_objects.h \
    $(top_srcdir)/include/blob.h \
    $(top_srcdir)/include/blob_2D.h \
    $(top_srcdir)/include/blob_2D_color.h \
    $(top_srcdir)/include/blobserver.h \
    $(top_srcdir)/include/configurator.h \
    $(top_srcdir)/include/capture.h \
    $(top_srcdir)/include/creator.h \
    $(top_srcdir)/include/descriptor_hog.h \
    $(top_srcdir)/include/actuator.h \
    $(top_srcdir)/include/actuator_armpcl.h \
    $(top_srcdir)/include/actuator_bgsubtractor.h \
    $(top_srcdir)/include/actuator_clusterPcl.h \
    $(top_srcdir)/include/actuator_depthtouch.h \
    $(top_srcdir)/include/actuator_fiducialtracker.h \
    $(top_srcdir)/include/actuator_hog.h \
    $(top_srcdir)/include/actuator_lightSpots.h \
    $(top_srcdir)/include/actuator_meanOutliers.h \
    $(top_srcdir)/include/actuator_objOnAPlane.h \
    $(top_srcdir)/include/actuator_stitch.h \
    $(top_srcdir)/include/actuator_nop.h \
	$(top_srcdir)/include/hdribuilder.h \
	$(top_srcdir)/include/helpers.h \
	$(top_srcdir)/include/shmpointcloud.h \
    $(top_srcdir)/include/source.h \
    $(top_srcdir)/include/source_2d.h \
    $(top_srcdir)/include/source_2d_opencv.h \
    $(top_srcdir)/include/source_2d_shmdata.h \
    $(top_srcdir)/include/source_3d_shmdata.h \
    $(top_srcdir)/include/threadPool.h

# External libs, compiled along blobserver
blobserver_SOURCES += \
    $(top_srcdir)/ext/libfidtrack/dump_graph.c \
    $(top_srcdir)/ext/libfidtrack/fidtrack120.c \
    $(top_srcdir)/ext/libfidtrack/fidtrackX.c \
    $(top_srcdir)/ext/libfidtrack/segment.c \
    $(top_srcdir)/ext/libfidtrack/threshold.c \
    $(top_srcdir)/ext/libfidtrack/tiled_bernsen_threshold.c \
    $(top_srcdir)/ext/libfidtrack/topologysearch.c \
    $(top_srcdir)/ext/libfidtrack/treeidmap.cpp

noinst_HEADERS += \
    $(top_srcdir)/ext/libfidtrack/default_trees.h \
    $(top_srcdir)/ext/libfidtrack/dump_graph.h \
    $(top_srcdir)/ext/libfidtrack/fidtrack120.h \
    $(top_srcdir)/ext/libfidtrack/fidtrackX.h \
    $(top_srcdir)/ext/libfidtrack/floatpoint.h \
    $(top_srcdir)/ext/libfidtrack/segment.h \
    $(top_srcdir)/ext/libfidtrack/threshold.h \
    $(top_srcdir)/ext/libfidtrack/tiled_bernsen_threshold.h \
    $(top_srcdir)/ext/libfidtrack/topologysearch.h \
    $(top_srcdir)/ext/libfidtrack/treeidmap.h

blobserver_CXXFLAGS = \
    -I../ext/libfidtrack \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS) \
    $(LCMS_CFLAGS) \
    $(MAPPER_CFLAGS) \
    $(PCL_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobserver_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS) \
    $(LCMS_LIBS) \
    $(MAPPER_LIBS) \
    $(PCL_LIBS) \
    $(SHMDATA_LIBS) \
    $(XML2_LIBS)

if PCL
blobserver_LDADD += $(BOOST_SYSTEM_LIBS)
endif

# Blobcontroler
blobcontroller_SOURCES = \
    base_objects.cpp \
    blobcontroller.cpp \
    configurator.cpp

blobcontroller_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS) \
    $(PCL_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobcontroller_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS) \
    $(PCL_LIBS) \
    $(SHMDATA_LIBS) \
    $(XML2_LIBS)

# Blobtrainer
blobtrainer_SOURCES = \
    blobtrainer.cpp \
    descriptor_hog.cpp

blobtrainer_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS)

blobtrainer_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS)
