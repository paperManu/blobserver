#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.68)
AC_INIT([BLOBSERVER],
        [0.6.1],
        [http://code.sat.qc.ca/redmine/projects/blobserver/issues],
        [blobserver],
        [http://code.sat.qc.ca/redmine/projects/blobserver])
AC_CONFIG_MACRO_DIR([m4])
AC_PROG_LIBTOOL
AM_INIT_AUTOMAKE()
AC_CONFIG_HEADERS([include/config.h])

AC_SUBST([LIBBLOBSERVER_SO_VERSION], [0:6:0])
AC_SUBST([LIBBLOBSERVER_API_VERSION], [0.6])
AC_DEFINE(LIBBLOBSERVER_API_VERSION, ["0.6"], [API version])

# Check for C and C++ compilers
AC_PROG_CXX
AM_PROG_CC_C_O

AC_MSG_CHECKING([to see if compiler understands -std=gnu++0x])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$CFLAGS -std=gnu++0x"
#  CXXFLAGS="-g3 -O0 -std=gnu++0x"
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([compiler does not understand -std=c++0x])
])

export PKG_CONFIG_PATH=$srcdir/pkgconfig:/usr/local/lib/pkgconfig:/usr/lib/pkgconfig

case "$host" in *-darwin* | *-macos10*)
  LIBTOOL="glibtool"
  CXXFLAGS+=" -D__Darwin"
  LDFLAGS+=" -undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CXXFLAGS+=" -I/usr/local/include -I/opt/local/include"
    #LDFLAGS+=" -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CXXFLAGS+=" -I/sw/include"
    #LDFLAGS+=" -L/sw/lib"
  fi
  # OSX does not include /usr/local/lib/pkgconfig path, so we need to
  # add it. 
  PKG_CONFIG_PATH=$srcdir/pkgconfig:/usr/local/lib/pkgconfig:/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  LDFLAGS+=" -fPIC -L/usr/local/lib -L/usr/local/lib64"
  ;;
esac

# Check for header files
AC_HEADER_STDC

# GLib
PKG_CHECK_MODULES([GLIB], glib-2.0, have_glib=true, have_glib=false)
if test "x${have_glib}" = "xfalse" ; then
    AC_MSG_ERROR([Missing glib])
fi

# OpenCV
PKG_CHECK_MODULES([OPENCV], [opencv >= 2.4])
if test "x${have_opencv}" = "xfalse" ; then
    AC_MSG_ERROR([Missing opencv])
fi

# Shmdata
PKG_CHECK_MODULES([SHMDATA], [shmdata-0.8 shmdata-any-0.8],
    AC_DEFINE([HAVE_SHMDATA], [1], [Defined to 1 if libshmdata is detected]),
    AC_DEFINE([HAVE_SHMDATA], [0], [Defined to 1 if libshmdata is detected]))

# PCL
PKG_CHECK_MODULES([PCL], [pcl_filters-1.7 pcl_common-1.7 pcl_octree-1.7 pcl_sample_consensus-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_surface-1.7 pcl_features-1.7 pcl_visualization-1.7 pcl_segmentation-1.7],
    AC_DEFINE([HAVE_PCL], [1], [Defined to 1 if pcl-1.7 is detected]),
    AC_DEFINE([HAVE_PCL], [0], [Defined to 1 if pcl-1.7 is detected]))
AM_CONDITIONAL(PCL, test -n "$PCL_LIBS")

# Boost (needed for PCL...)
BOOST_REQUIRE([1.48])
BOOST_SYSTEM

# Liblo
PKG_CHECK_MODULES([LIBLO], [liblo >= 0.26])
if test "x${have_liblo}" = "xfalse" ; then
    AC_MSG_ERROR([Missing liblo])
fi

# LCMS
PKG_CHECK_MODULES([LCMS], [lcms2 >= 2.2])
if test "x${have_lcms}" = "xfalse" ; then
    AC_MSG_ERROR([Missing lcms])
fi

# LibAtom
PKG_CHECK_MODULES([LIBATOM], atom-0.2)
if test "x${have_atom}" = "xfalse" ; then
    AC_MSG_ERROR([Missing atom])
fi

# LibXML2
PKG_CHECK_MODULES([XML2], libxml-2.0)
if test "x${have_xml2}" = "xfalse" ; then
    AC_MSG_ERROR([Missing libxml2])
fi

# LibMapper
PKG_CHECK_MODULES([MAPPER], libmapper-0,
    AC_DEFINE([HAVE_MAPPER], [1], [Defined to 1 if libmapper is detected]),
    AC_DEFINE([HAVE_MAPPER], [0], [Defined to 0 if libmapper is detected]))

# Aravis / Genicam / Gige cameras
PKG_CHECK_MODULES([ARAVIS], aravis-0.4,
    AC_DEFINE([HAVE_ARAVIS], [1], [Defined to 1 if aravis is detected]),
    AC_DEFINE([HAVE_ARAVIS], [0], [Defined to 0 if aravis is detected]))

# Python
PKG_CHECK_MODULES([PYTHON], python3,
    AC_DEFINE([HAVE_PYTHON], [1], [Defined to 1 if python3 is detected]),
    AC_DEFINE([HAVE_PYTHON], [0], [Defined to 1 if python3 is detected]))
AM_CONDITIONAL(PYTHON, test -n "$PYTHON_LIBS")

# GNU help2man creates man pages from --help output; in many cases, this
# is sufficient, and obviates the need to maintain man pages separately.
# However, this means invoking executables, which we generally cannot do
# when cross-compiling, so we test to avoid that (the variable
# "cross_compiling" is set by AC_PROG_CC).
if test $cross_compiling = no; then
    AM_MISSING_PROG(HELP2MAN, help2man)
else
    HELP2MAN=:
fi

AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test "x$DOXYGEN" = "xtrue" ; then
    AC_MSG_WARN([Doxygen not found - documentation will not be generated])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])
AM_COND_IF([HAVE_DOXYGEN], [AC_CONFIG_FILES([Doxyfile])])

AC_CONFIG_FILES([
    Makefile
    blobserver.pc
    ext/Makefile
    ext/libfidtrack/Makefile
    src/Makefile
    plugins/Makefile
    plugins/actuators/Makefile
    man/Makefile
    ])
AC_OUTPUT

