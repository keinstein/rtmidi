ACLOCAL_AMFLAGS = -I m4 -I rtmidi-m4 --install
AM_CFLAGS = $(CODE_COVERAGE_CFLAGS)
AM_CXXFLAGS = $(CODE_COVERAGE_CFLAGS)
AM_LDFLAGS = $(CODE_COVERAGE_LDFLAGS)
LIBS = $(CODE_COVERAGE_LIBS)
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

gcov:
	$(GCOV) -p -s "$(srcdir)" -o "$$dir" $$(find . -name '*.gc*')
