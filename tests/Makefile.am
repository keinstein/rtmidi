
### RtMidi tests Makefile - for various flavors of unix
#RTMIDITESTCXXFLAGS += -I$(top_srcdir)/%D%
#RTMIDITESTLDFLAGS  += -L$(top_builddir)/%D% -lrtmidi@rtmidi_suffix@ @LIBS@

check_PROGRAMS += \
	%D%/midiprobe \
	%D%/midiout \
	%D%/qmidiin \
	%D%/cmidiin \
	%D%/sysextest \
	%D%/midiprobe2 \
	%D%/midiprobe-all \
	%D%/cmidiin2 \
	%D%/qmidiin2 \
	%D%/midiout2 \
	%D%/loopback \
	%D%/errors \
	%D%/midiclock_in \
	%D%/midiclock_out \
	%D%/lostportdescriptor \
	%D%/testequalityoperator \
	%D%/apinames \
	%D%/sequencerportlist \
	%D%/copyonwrite

TESTS += \
	%D%/midiprobe \
	%D%/midiprobe2 \
	%D%/midiprobe-all \
	%D%/errors \
	%D%/lostportdescriptor \
	%D%/testequalityoperator \
	%D%/apinames \
	%D%/sequencerportlist \
	%D%/copyonwrite

CLEANFILES += \
	%D%/*.class

EXTRA_DIST += \
	%D%/cmidiin.dsp \
	%D%/midiout.dsp \
	%D%/midiprobe.dsp \
	%D%/qmidiin.dsp	\
	%D%/sysextest.dsp \
	%D%/RtMidi.dsw

if HAVE_EXTERNAL_LOOPBACK
TESTS += \
	%D%/run-sysextest \
	%D%/run-qmidiio \
	%D%/run-cmidiio \
	%D%/run-qmidiio2 \
	%D%/run-cmidiio2 \
	%D%/run-midiclockio
endif

if RTMIDI_HAVE_VIRTUAL_DEVICES
TESTS += %D%/loopback
endif

RTMIDICOMMONTESTSOURCES = \
	%D%/test-common.cpp \
	%D%/test-common.h


%C%_midiprobe_SOURCES      = %D%/midiprobe.cpp
%C%_midiout_SOURCES        = %D%/midiout.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_qmidiin_SOURCES        = %D%/qmidiin.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_cmidiin_SOURCES        = %D%/cmidiin.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_sysextest_SOURCES      = %D%/sysextest.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_midiprobe2_SOURCES     = %D%/midiprobe2.cpp
%C%_midiprobe_all_SOURCES  = %D%/midiprobe-all.cpp
%C%_cmidiin2_SOURCES       = %D%/cmidiin2.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_qmidiin2_SOURCES       = %D%/qmidiin2.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_midiout2_SOURCES       = %D%/midiout2.cpp
%C%_loopback_SOURCES       = %D%/loopback.cpp
%C%_errors_SOURCES         = %D%/errors.cpp
%C%_midiclock_in_SOURCES   = %D%/midiclock.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_midiclock_out_SOURCES  = %D%/midiclock.cpp $(RTMIDICOMMONTESTSOURCES)
%C%_lostportdescriptor_SOURCES       = %D%/lostportdescriptor.cpp
%C%_testequalityoperator_SOURCES       = %D%/testequalityoperator.cpp
%C%_apinames_SOURCES       = %D%/apinames.cpp
%C%_sequencerportlist_SOURCES       = %D%/sequencerportlist.cpp
%C%_copyonwrite_SOURCES       = %D%/copyonwrite.cpp

# When a nonstandard gettext library or wrapper is used,
# we need extra flags.
%C%_midiprobe_CXXFLAGS     = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiout_CXXFLAGS       = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_qmidiin_CXXFLAGS       = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_cmidiin_CXXFLAGS       = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_sysextest_CXXFLAGS     = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiprobe2_CXXFLAGS    = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiprobe_all_CXXFLAGS = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_cmidiin2_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_qmidiin2_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiout2_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_loopback_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_errors_CXXFLAGS        = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiclock_in_CXXFLAGS  = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_midiclock_out_CXXFLAGS = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_lostportdescriptor_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_testequalityoperator_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_apinames_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_sequencerportlist_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED
%C%_copyonwrite_CXXFLAGS      = $(AM_CXXFLAGS) $(RTMIDITESTCXXFLAGS) -DRTMIDI_NO_WARN_DEPRECATED


%C%_midiprobe_LDFLAGS      = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiout_LDFLAGS        = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_qmidiin_LDFLAGS        = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_cmidiin_LDFLAGS        = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_sysextest_LDFLAGS      = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiprobe2_LDFLAGS     = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiprobe_all_LDFLAGS  = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_cmidiin2_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_qmidiin2_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiout2_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_loopback_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_errors_LDFLAGS         = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiclock_in_LDFLAGS   = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_midiclock_out_LDFLAGS  = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_lostportdescriptor_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_testequalityoperator_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_apinames_LDFLAGS       = $(AM_LDFLAGS) $(RTMIDITESTLDFLAGS)
%C%_sequencerportlist_LDFLAGS       = $(AM_LDFLAGS) $(ALSA_LIBS) $(JACK_LIBS) #$(RTMIDITESTLDFLAGS)
%C%_copyonwrite_LDFLAGS       = $(AM_LDFLAGS) $(ALSA_LIBS) $(JACK_LIBS) #$(RTMIDITESTLDFLAGS)


%C%_midiprobe_LDADD      = $(RTMIDILIBRARYNAME)
%C%_midiout_LDADD        = $(RTMIDILIBRARYNAME)
%C%_qmidiin_LDADD        = $(RTMIDILIBRARYNAME)
%C%_cmidiin_LDADD        = $(RTMIDILIBRARYNAME)
%C%_sysextest_LDADD      = $(RTMIDILIBRARYNAME)
%C%_midiprobe2_LDADD     = $(RTMIDILIBRARYNAME)
%C%_midiprobe_all_LDADD  = $(RTMIDILIBRARYNAME)
%C%_cmidiin2_LDADD       = $(RTMIDILIBRARYNAME)
%C%_qmidiin2_LDADD       = $(RTMIDILIBRARYNAME)
%C%_midiout2_LDADD       = $(RTMIDILIBRARYNAME)
%C%_loopback_LDADD       = $(RTMIDILIBRARYNAME)
%C%_errors_LDADD         = $(RTMIDILIBRARYNAME)
%C%_midiclock_in_LDADD   = $(RTMIDILIBRARYNAME)
%C%_midiclock_out_LDADD  = $(RTMIDILIBRARYNAME)
%C%_lostportdescriptor_LDADD       = $(RTMIDILIBRARYNAME)
%C%_testequalityoperator_LDADD       = $(RTMIDILIBRARYNAME)
%C%_apinames_LDADD       = $(RTMIDILIBRARYNAME)
%C%_sequencerportlist_LDADD       = #$(RTMIDILIBRARYNAME)
%C%_copyonwrite_LDADD       = #$(RTMIDILIBRARYNAME)


if RTMIDICOPYDLLS

#-------------------------------------------------------------------------------------
# Installing DLLs
#-------------------------------------------------------------------------------------



rtmidi_installdll:
	@echo 'solving references for $(DLLLINKFILE)... '
	set -e -x; \
	DLLSEARCHPATH="$(DLLSEARCHPATH)$(LOCALDLLSEARCHPATH)" ; \
	for f in `LANG=C $(OBJDUMP) -p  $(DLLEXEDIR)/$(DLLLINKFILE) |sed '/^\s*DLL Name:.*\(lib\|thread\|wx\|mingw\|gcc\|stdc++\)/ { s/^\s*DLL Name:\s*//; p } ; d '`; \
	do \
		echo -n checking "$$f ... " ; \
		if [ ! -f $(DLLEXEDIR)/$$f ] ; then \
			echo -n "searching... " ; \
			IFS=: ; \
			for d in $$DLLSEARCHPATH ; do \
				echo "looking in $$d" ; \
				IFS= ; \
				fullname="$$d/$$f" ; \
				echo "testing $$fullname" ; \
				if test -f "$$fullname" ; \
				then \
					if cmp "$$fullname" "`pwd`/$(DLLEXEDIR)/$$f"; \
					then \
						echo "ok." ;\
					else \
						echo "installing $$fullname " ; \
						$(INSTALL_PROGRAM_ENV) $(INSTALL_DATA) "$$fullname" "$(DLLEXEDIR)" ; \
					fi ; \
					$(MAKE) $(AM_MAKEFLAGS) DLLLINKFILE="$$f" DLLEXEDIR="$(DLLEXEDIR)" rtmidi_installdll ; \
				elif test -f "$${fullname}.gz" ; \
				then \
					if cmp "$${fullname}.gz" "`pwd`/$(DLLEXEDIR)/$${f}.gz"; \
					then \
						echo "ok." ;\
					else \
						echo "installing $$fullname " ; \
						$(INSTALL_PROGRAM_ENV) $(INSTALL_DATA) "$${fullname}.gz" "$(DLLEXEDIR)" ; \
					fi; \
					GZIP=$(GZIP_ENV) gzip -dc $(DLLEXEDIR)/`basename "$${fullname}.gz"` >$(DLLEXEDIR)/"$$f" ; \
					$(MAKE) $(AM_MAKEFLAGS) DLLLINKFILE="$$f" DLLEXEDIR="$(DLLEXEDIR)" rtmidi_installdll ; \
				fi ; \
			done ; \
		fi ; \
		echo "done." ;\
	done ;

#-------------------------------------------------------------------------------------
# Certain fixed files
#-------------------------------------------------------------------------------------



check-am: check-dll-rtmidi

check-dll-rtmidi: all-am
	$(MAKE) $(AM_MAKEFLAGS) $(check_PROGRAMS)
	$(MAKE) $(AM_MAKEFLAGS)	linkchecks-rtmidi

linkchecks-rtmidi:
	set -e -x ; \
	for d in `echo $(DLLPROGRAMS)| tr ' ' '\n' | grep -i -e '$(EXEEXT)$$\|.dll$$'` ; \
	do \
		ls -l "$$d" ; \
		file=`basename "$$d"` ; \
		dir=`dirname "$$d"` ; \
		$(MAKE) $(AM_MAKEFLAGS) DLLLINKFILE="$$file" DLLEXEDIR="$$dir" rtmidi_installdll ; \
		if test -f "$$dir/.libs/$$file"; \
		then \
			$(MAKE) $(AM_MAKEFLAGS) DLLLINKFILE="$$file" DLLEXEDIR="$$dir/.libs" rtmidi_installdll ; \
		fi ; \
	done
endif

