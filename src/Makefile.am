bin_PROGRAMS = blobserver

blobserver_SOURCES = \
    main.cpp \
    blob.cpp \
    blob_lightSpot.cpp

noinst_HEADERS = \
    blob.h \
    blob_lightSpot.h

blobserver_CXXFLAGS = \
    -std=gnu++0x \
	$(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) 

blobserver_LDADD = \
	$(GLIB_LIBS) \
    $(OPENCV_LIBS) \
    $(LIBLO_LIBS) 
