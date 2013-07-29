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
    configurator.cpp \
    descriptor_hog.cpp \
    detector.cpp \
    detector_bgsubtractor.cpp \
	detector_depthtouch.cpp \
    detector_hog.cpp \
    detector_lightSpots.cpp \
    detector_meanOutliers.cpp \
    detector_objOnAPlane.cpp \
    detector_nop.cpp \
	hdribuilder.cpp \
    source.cpp \
    source_2d.cpp \
    source_2d_opencv.cpp \
    source_2d_shmdata.cpp \
    threadPool.cpp

noinst_HEADERS = \
    $(top_srcdir)/include/abstract-factory.h \
    $(top_srcdir)/include/abstract-factory_spec.h \
    $(top_srcdir)/include/base_objects.h \
    $(top_srcdir)/include/blob.h \
    $(top_srcdir)/include/blob_2D.h \
    $(top_srcdir)/include/configurator.h \
    $(top_srcdir)/include/capture.h \
    $(top_srcdir)/include/creator.h \
    $(top_srcdir)/include/descriptor_hog.h \
    $(top_srcdir)/include/detector.h \
    $(top_srcdir)/include/detector_bgsubtractor.h \
    $(top_srcdir)/include/detector_depthtouch.h \
    $(top_srcdir)/include/detector_hog.h \
    $(top_srcdir)/include/detector_lightSpots.h \
    $(top_srcdir)/include/detector_meanOutliers.h \
    $(top_srcdir)/include/detector_objOnAPlane.h \
    $(top_srcdir)/include/detector_nop.h \
	$(top_srcdir)/include/hdribuilder.h \
	$(top_srcdir)/include/helpers.h \
    $(top_srcdir)/include/source.h \
    $(top_srcdir)/include/source_2d.h \
    $(top_srcdir)/include/source_2d_opencv.h \
    $(top_srcdir)/include/source_2d_shmdata.h \
    $(top_srcdir)/include/threadPool.h

blobserver_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS) \
    $(LCMS_CFLAGS) \
    $(PCL_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobserver_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS) \
    $(LCMS_LIBS) \
    $(PCL_LIBS) \
    $(SHMDATA_LIBS) \
    $(XML2_LIBS)

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
