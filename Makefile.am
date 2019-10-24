ACLOCAL_AMFLAGS = -I m4 -I rtmidi-m4 --install
AM_CFLAGS = $(CODE_COVERAGE_CFLAGS)
AM_CXXFLAGS = $(CODE_COVERAGE_CXXFLAGS)
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

#@CODE_COVERAGE_RULES@

include %D%/Makefile.library

gcov:
	$(GCOV) -p -s "$(srcdir)" -o "$$dir" $$(find . -name '*.gc*')
