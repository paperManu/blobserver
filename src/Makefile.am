bin_PROGRAMS = blobserver

blobserver_SOURCES = \
    main.cpp \
    blob.cpp \
    blob_2D.cpp \
    detector.cpp \
    source.cpp \
    source_opencv.cpp

noinst_HEADERS = \
    blob.h \
    blob_2D.h \
    detector.h \
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
