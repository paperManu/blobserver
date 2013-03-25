bin_PROGRAMS = \
    blobserver \
    blobcontroller

# Blobserver
blobserver_SOURCES = \
    base_objects.cpp \
    blobserver.cpp \
    blob.cpp \
    blob_2D.cpp \
    configurator.cpp \
    detector.cpp \
    detector_meanOutliers.cpp \
    detector_lightSpots.cpp \
    detector_objOnAPlane.cpp \
    detector_nop.cpp \
	hdribuilder.cpp \
    source.cpp \
    source_opencv.cpp \
    source_shmdata.cpp \
    threadPool.cpp

noinst_HEADERS = \
    abstract-factory.h \
    abstract-factory_spec.h \
    base_objects.h \
    blob.h \
    blob_2D.h \
    configurator.h \
    creator.h \
    detector.h \
    detector_meanOutliers.h \
    detector_lightSpots.h \
    detector_objOnAPlane.h \
    detector_nop.h \
	hdribuilder.h \
    source.h \
    source_opencv.h \
    source_shmdata.h \
    threadPool.h

blobserver_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS) \
    $(LCMS_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobserver_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS) \
    $(LCMS_LIBS) \
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
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobcontroller_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS) \
    $(SHMDATA_LIBS) \
    $(XML2_LIBS)
