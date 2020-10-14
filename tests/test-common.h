//*****************************************//
//  by Gary Scavone, 2003-2005.
/*!  \file 

  program to test MIDI sysex sending and receiving.
*/
//*****************************************//

#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <signal.h>


// Platform-dependent sleep routines.
#if defined(WIN32)
  #include <windows.h>
#else // Unix variants
#include <time.h>
#endif

inline bool SLEEP(unsigned long long int  milliseconds ) {
#if defined(WIN32)
  Sleep( (DWORD) milliseconds );
  return true;
#else
  struct timespec time,time2;
  time.tv_sec = milliseconds / 1000;
  time.tv_nsec = (milliseconds % 1000) * 1000000;
  int status = nanosleep(&time,&time2);
  return !status;
  /*
  if ((status)) {
    int error = errno;
    std::perror("Sleep has been interrupted");
    exit(error);
  }
  */
#endif
}

// This function should be embedded in a try/catch block in case of
// an exception.  It offers the user a choice of MIDI ports to open.
// It returns false if there are no ports available.
bool chooseMidiPort( RtMidi *rtmidi );
void usage ( const char * name,
             rtmidi::PortList & list,
             rtmidi::PortDescriptor::PortCapabilities flags );
bool openMidiPort( const char * name,
                   rtmidi::Midi & midi,
                   rtmidi::PortList list,
                   rtmidi::PortDescriptor::PortCapabilities flags );
bool chooseMidiPort( rtmidi::Midi & midi,
                     rtmidi::PortList list,
                     rtmidi::PortDescriptor::PortCapabilities flags );

