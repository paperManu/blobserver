bin_PROGRAMS = blobserver

blobserver_SOURCES = \
    main.cpp \
    blob.cpp \
    blob_2D.cpp \
    detector.cpp \
    detector_meanOutliers.cpp \
    detector_lightSpots.cpp \
    detector_objOnAPlane.cpp \
    source.cpp \
    source_opencv.cpp

noinst_HEADERS = \
    blob.h \
    blob_2D.h \
    detector.h \
    detector_meanOutliers.h \
    detector_lightSpots.h \
    detector_objOnAPlane.h \
    source.h \
    source_opencv.h

blobserver_CXXFLAGS = \
    -std=gnu++0x \
	$(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS)

blobserver_LDADD = \
	$(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) \
    $(LIBATOM_LIBS)
