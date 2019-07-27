//*****************************************//
//  qmidiin.cpp
//  by Gary Scavone, 2003-2004.
//
//  Simple program to test MIDI input and
//  retrieval from the queue.
//
//*****************************************//

#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <signal.h>

// Platform-dependent sleep routines.
#if defined(WIN32)
  #include <windows.h>
  #define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds ) 
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

void usage( void ) {
  // Error function in case of incorrect command-line
  // argument specifications.
  std::cout << "\nusage: qmidiin <port>\n";
  std::cout << "    where port = the device to use (default = 0).\n\n";
  exit( 0 );
}

int main( int argc, char *argv[] )
{
  RtMidiIn *midiin = 0;
  std::vector<unsigned char> message;
  int nBytes, i;
  double stamp;

  // Minimal command-line check.
  if ( argc > 2 ) usage();

  // RtMidiIn constructor
  try {
    midiin = new RtMidiIn();
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    exit( EXIT_FAILURE );
  }

  // Check available ports vs. specified.
  unsigned int port = 0;
  unsigned int nPorts = midiin->getPortCount();
  if ( argc == 2 ) port = (unsigned int) atoi( argv[1] );
  if ( port >= nPorts ) {
    delete midiin;
    std::cout << "Invalid port specifier!\n";
    usage();
  }

  try {
    midiin->openPort( port );
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  // Don't ignore sysex, timing, or active sensing messages.
  midiin->ignoreTypes( false, false, false );

  // Install an interrupt handler function.
  done = false;
  (void) signal(SIGINT, finish);

  // Periodically check input queue.
  std::cout << "Reading MIDI from port " << midiin->getPortName() << " ... quit with Ctrl-C.\n";
  while ( !done ) {
    try {
      stamp = midiin->getMessage( &message );
    } catch (const rtmidi::Error & e) {
      e.printMessage();
      break;
    }

    nBytes = message.size();
    for ( i=0; i<nBytes; i++ )
      std::cout << "Byte " << i << " = " << (int)message[i] << ", ";
    if ( nBytes > 0 )
      std::cout << "stamp = " << stamp << std::endl;

    // Sleep for 10 milliseconds.
    SLEEP( 10 );
  }

  // Clean up
 cleanup:
  delete midiin;

  return 0;
}
