#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.68)
AC_INIT(BARYCENTER, 0.0.9, emmanueldurand@gmail.com)
AC_CONFIG_MACRO_DIR([m4])
AC_PROG_LIBTOOL
AM_INIT_AUTOMAKE()

# Check for C and C++ compilers
AC_PROG_CXX
AC_PROG_CC

# Check for header files
AC_HEADER_STDC

# OpenCV
PKG_CHECK_MODULES([OPENCV], [opencv >= 2.4])
if test "x${have_opencv}" = "xfalse" ; then
    AC_MSG_ERROR([Missing opencv])
fi

# Liblo
PKG_CHECK_MODULES([LIBLO], [liblo >= 0.26])
if test "x${have_liblo}" = "xfalse" ; then
    AC_MSG_ERROR([Missing liblo])
fi

AC_CONFIG_FILES([
    Makefile
    src/Makefile
    man/Makefile
    ])
AC_OUTPUT
