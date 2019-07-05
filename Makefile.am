ACLOCAL_AMFLAGS = -I m4 -I rtmidi-m4 --install
SUBDIRS = po
if MAKE_DOC
SUBDIRS += doc
endif
RTMIDICONFIGRPATH = config/config.rpath
DLLPROGRAMS = $(check_PROGRAMS)
check_PROGRAMS =
TESTS =
CLEANFILES =
DISTCLEANFILES =

include %D%/Makefile.library

MULTITARGETS = all check

multitarget:
	for t in $(MULTITARGETS) ; do $(MAKE) $$t ; done
