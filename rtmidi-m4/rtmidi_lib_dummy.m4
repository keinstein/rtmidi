# AC_LIB_DUMMY(FUNCTION,
#              [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND],
#              [OTHER-LIBRARIES])
# ------------------------------------------------------
#
# Use a cache variable name containing both the library and function name,
# because the test really is for library $1 defining function $2, not
# just for library $1.  Separate tests with the same $1 and different $2s
# may have different results.
#
# Note that using directly AS_VAR_PUSHDEF([ac_Lib], [ac_cv_lib_$1_$2])
# is asking for troubles, since AC_CHECK_LIB($lib, fun) would give
# ac_cv_lib_$lib_fun, which is definitely not what was meant.  Hence
# the AS_LITERAL_IF indirection.
#
# FIXME: This macro is extremely suspicious.  It DEFINEs unconditionally,
# whatever the FUNCTION, in addition to not being a *S macro.  Note
# that the cache does depend upon the function we are looking for.
#
# It is on purpose we used `ac_check_lib_save_LIBS' and not just
# `ac_save_LIBS': there are many macros which don't want to see `LIBS'
# changed but still want to use AC_CHECK_LIB, so they save `LIBS'.
# And ``ac_save_LIBS' is too tempting a name, so let's leave them some
# freedom.
AC_DEFUN([RTMIDI_LIB_DUMMY],[
	AC_LANG_PUSH(C++)
	rtmidi_have_dummy=yes
	RTMIDI_API="$RTMIDI_API -D__RTMIDI_DUMMY__"
	RTMIDI_LIB_CFLAGS="$RTMIDI_LIB_CFLAGS $DUMMY_CFLAGS"
	RTMIDI_LIBS="$RTMIDI_LIBS $DUMMY_LIBS"
	RTMIDI_HAVE_VIRTUAL_DEVICES=yes
	$1
	AC_LANG_POP(C++)
])dnl RTMIDI_LIB_DUMMY
