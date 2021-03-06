//*****************************************//
//  qmidiin.cpp
//  by Gary Scavone, 2003-2014.
//  and Tobias Schlemmer, 2014
//
/*! \example qmidiin2.cpp
  Simple program to test MIDI input and
  retrieval from the queue.
*/
//*****************************************//

#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <signal.h>

// Platform-dependent sleep routines.
#if defined(WIN32)
#include <windows.h>
#define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds )
#undef UNIQUE_PORT_NAME
#else // Unix variants
#include <time.h>
inline void SLEEP(unsigned long long int  milliseconds ) {
  struct timespec time,time2;
  time.tv_sec = milliseconds / 1000;
  time.tv_nsec = (milliseconds % 1000) * 1000000;
  int status;
  if ((status = nanosleep(&time,&time2))) {
    int error = errno;
    std::perror("Sleep has been interrupted");
    exit(error);
  }
}
#endif

bool done;
static void finish( int /*ignore*/ ){ done = true; }

void usage( rtmidi::PortList list ) {
  // Error function in case of incorrect command-line
  // argument specifications.
  std::cout << "\nusage: qmidiin <port>\n";
  std::cout << "    where port = the device to use (default = first available port).\n\n";

  std::cout << "Available ports:" << std::endl;
  int flags = rtmidi::PortDescriptor::SESSION_PATH |
    rtmidi::PortDescriptor::UNIQUE_PORT_NAME |
    rtmidi::PortDescriptor::INCLUDE_API;
  for (rtmidi::PortList::iterator i = list.begin();
       i != list.end(); i++) {
    std::cout << "\""
	      << (*i)->getName(rtmidi::PortDescriptor::SESSION_PATH |
			       rtmidi::PortDescriptor::UNIQUE_PORT_NAME |
			       rtmidi::PortDescriptor::INCLUDE_API)
	      << "\"";
    std::cout << "\t";
    std::cout << (*i)->getName() << std::endl;
  }
  exit( 0 );
}

int main( int argc, char *argv[] )
{
  std::vector<unsigned char> message;
  int nBytes, i;
  double stamp;


  // rtmidi::MidiIn constructor
  try {
    rtmidi::MidiIn midiin(rtmidi::ALL_API);


    rtmidi::PortList list = midiin.getPortList();


    // Minimal command-line check.
    if ( argc > 2 ) usage(list);

    rtmidi::Pointer<rtmidi::PortDescriptor> port = 0;
    // Check available ports vs. specified.
    if ( argc == 2 ) {
      for (rtmidi::PortList::iterator i = list.begin();
	   i != list.end(); i++) {
	if (argv[1] == (*i)->getName(rtmidi::PortDescriptor::SESSION_PATH |
				     rtmidi::PortDescriptor::UNIQUE_PORT_NAME |
				     rtmidi::PortDescriptor::INCLUDE_API)) {
	  port = *i;
	  break;
	}
      }
    } else {
      port = list.front();
    }
    if ( !port ) {
      std::cout << "Invalid port specifier!\n";
      usage(list);
    }

    try {
      midiin.openPort( port );
    }
    catch ( rtmidi::Error &error ) {
      error.printMessage();
      return 1;
    }

    // Don't ignore sysex, timing, or active sensing messages.
    midiin.ignoreTypes( false, false, false );

    // Install an interrupt handler function.
    done = false;
    (void) signal(SIGINT, finish);

    // Periodically check input queue.
    std::cout << "Reading MIDI from port ... quit with Ctrl-C.\n";
    while ( !done ) {
      stamp = midiin.getMessage( &message );
      nBytes = message.size();
      for ( i=0; i<nBytes; i++ )
	std::cout << "Byte " << i << " = " << (int)message[i] << ", ";
      if ( nBytes > 0 )
	std::cout << "stamp = " << stamp << std::endl;

      // Sleep for 10 milliseconds.
      SLEEP( 10 );
    }
  }
  catch ( rtmidi::Error &error ) {
    error.printMessage();
    exit( EXIT_FAILURE );
  }

  return 0;
}
