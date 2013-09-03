ACLOCAL_AMFLAGS = -I m4
SUBDIRS = \
    ext \
    man \
    src \
	plugins

if HAVE_DOXYGEN
dist_man_MANS = $(top_builddir)/man
$(top_srcdir)/man: doxyfile.stamp

doxyfile.stamp:
	$(DOXYGEN) Doxyfile
	echo Timestamp > doxyfile.stamp

CLEANFILES = doxyfile.stamp

doc: doxyfile.stamp
clean-local:
	rm -rf $(top_builddir)/man/html
endif
