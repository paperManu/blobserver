AM_CPPFLAGS = \
	-DLIBDIR=\"$(libdir)\" \
    -I$(top_srcdir)/include \
    -DG_LOG_DOMAIN="" \
    -DG_DISABLE_CHECKS

bin_PROGRAMS = \
    blobserver \
    blobcontroller \
    blobcrop \
    blobtrainer

# Blobserver
blobserver_SOURCES = \
    base_objects.cpp \
    blobserver.cpp \
    blob.cpp \
    blob_2D.cpp \
    blob_2D_color.cpp \
    configurator.cpp \
    actuator.cpp \
	hdribuilder.cpp \
    source.cpp \
    source_2d.cpp \
    source_2d_gige.cpp \
    source_2d_image.cpp \
    source_2d_opencv.cpp \
    source_2d_shmdata.cpp \
    source_3d_shmdata.cpp \
    threadPool.cpp

blobserverdir = $(prefix)/include/blobserver-@LIBBLOBSERVER_API_VERSION@

blobserver_HEADERS = \
    $(top_srcdir)/include/abstract-factory.h \
    $(top_srcdir)/include/abstract-factory_spec.h \
    $(top_srcdir)/include/actuator.h \
    $(top_srcdir)/include/blob.h \
    $(top_srcdir)/include/blob_2D.h \
    $(top_srcdir)/include/blob_2D_color.h \
    $(top_srcdir)/include/capture.h \
    $(top_srcdir)/include/creator.h \
	$(top_srcdir)/include/helpers.h \
    $(top_srcdir)/include/source.h

noinst_HEADERS = \
    $(top_srcdir)/include/base_objects.h \
    $(top_srcdir)/include/blobserver.h \
    $(top_srcdir)/include/configurator.h \
    $(top_srcdir)/include/constants.h \
	$(top_srcdir)/include/hdribuilder.h \
	$(top_srcdir)/include/shmpointcloud.h \
    $(top_srcdir)/include/source_2d.h \
    $(top_srcdir)/include/source_2d_gige.h \
    $(top_srcdir)/include/source_2d_image.h \
    $(top_srcdir)/include/source_2d_opencv.h \
    $(top_srcdir)/include/source_2d_shmdata.h \
    $(top_srcdir)/include/source_3d_shmdata.h \
    $(top_srcdir)/include/threadPool.h

blobserver_CXXFLAGS = \
    $(ARAVIS_CFLAGS) \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(LIBATOM_CFLAGS) \
    $(LCMS_CFLAGS) \
    $(MAPPER_CFLAGS) \
    $(PCL_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(XML2_CFLAGS)

blobserver_LDFLAGS = \
	-rdynamic \
	-ldl \
    -export-dynamic

blobserver_LDADD = \
    $(ARAVIS_LIBS) \
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

# Blobcrop
blobcrop_SOURCES = blobcrop.cpp
blobcrop_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(OPENCV_CFLAGS)

blobcrop_LDADD = \
    $(GLIB_LIBS) \
    $(OPENCV_LIBS)

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
