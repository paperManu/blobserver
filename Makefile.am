ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src man

if HAVE_DOXYGEN
directory = $(top_srcdir)/doc/

dist_man_MANS = $(directory)/man
$(directory)/man: doxyfile.stamp

doxyfile.stamp:
	$(DOXYGEN) Doxyfile
	echo Timestamp > doxyfile.stamp

CLEANFILES = doxyfile.stamp

doc: doxyfile.stamp
clean-local:
	rm -rf $(top_srcdir)/doc/man/*
endif
