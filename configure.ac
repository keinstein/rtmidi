# Process this file with autoconf to produce a configure script.
#AC_INIT(RtMidi, 2.1.0, gary@music.mcgill.ca, rtmidi)

m4_include(configure.version)

m4_define([ac_rtmidi_name],[RtMidi]ac_rtmidi_suffix)
m4_define([ac_rtmidi_package_name],[rtmidi]ac_rtmidi_suffix)

AC_INIT(ac_rtmidi_name, ac_rtmidi_version, [keinstein@users.sf.net], ac_rtmidi_package_name)
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR(RtMidi.cpp)
AC_CONFIG_FILES([Makefile doc/Makefile po/Makefile.in rtmidi-config rtmidi.pc  doc/doxygen/Doxyfile],[chmod oug+x rtmidi-config])
AM_INIT_AUTOMAKE([1.14 -Wall -Werror foreign subdir-objects])


# Fill GXX with something before test.
GXX="no"

# Clear the default CFLAGS and CXXFLAGS which will be set to “-g -O2” if unset.
: ${CFLAGS=""}
: ${CXXFLAGS=""}

# Enable some nice automake features if they are available
m4_ifdef([AM_MAINTAINER_MODE], [AM_MAINTAINER_MODE])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])


AC_CONFIG_MACRO_DIR([m4])

rtmidi_standalone="yes"
SUBDIRS="$SUBDIRS %D%/po"
rtmidi_subdir="$srcdir"
m4_define(rtmididir,[.])
m4_include(configure.library)


AC_LANG_PUSH(C++)
AX_CODE_COVERAGE
AC_CHECK_LIB(gcov,[__gcov_init], [], [
  AC_MSG_NOTICE([Cannot link gcov library trying without])
  CODE_COVERAGE_LIBS=""
  CODE_COVERAGE_LDFLAGS=""
])
AC_LANG_POP(C++)


AC_OUTPUT

