//*****************************************//
//  cmidiin.cpp
//  by Gary Scavone, 2003-2004.
//
//  Simple program to test MIDI input and
//  use of a user callback function.
//
//*****************************************//

#include "test-common.h"
#include "RtMidi.h"
#include <iostream>
#include <cstdlib>

std::atomic<bool> done;
static void finish( int /*ignore*/ ){
  done = true;
}
struct sigaction finishaction;

void usage( void ) {
  // Error function in case of incorrect command-line
  // argument specifications.
  std::cout << "\nuseage: cmidiin <port>\n";
  std::cout << "    where port = the device to use (default = 0).\n\n";
  exit( 0 );
}

void mycallback( double deltatime, std::vector< unsigned char > *message, void * /* userData */ )
{
  unsigned int nBytes = message->size();
  for ( unsigned int i=0; i<nBytes; i++ )
    std::cout << "Byte " << i << " = " << (int)message->at(i) << ", ";
  if ( nBytes > 0 )
    printf("stamp = %.2f\n", deltatime);
}

int main( int argc, char *argv[] )
{
  RtMidiIn *midiin = 0;

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

  try {

    if ( argc == 2 ) {
      printf("noochooseidiport\n");
      // Check available ports vs. specified.
      unsigned int port = 0;
      unsigned int nPorts = midiin->getPortCount();
      port = (unsigned int) atoi( argv[1] );
      if ( port >= nPorts ) {
        delete midiin;
        std::cout << "Invalid port specifier!\n";
        usage();
      }

      midiin->openPort( port );
    } else {
      printf("choose midi port\n");
      chooseMidiPort(midiin);
    }

    // Set our callback function.  This should be done immediately after
    // opening the port to avoid having incoming messages written to the
    // queue instead of sent to the callback function.
    midiin->setCallback( &mycallback );

    // Don't ignore sysex, timing, or active sensing messages.
    midiin->ignoreTypes( false, false, false );

    // Install an interrupt handler function.
    std::cout << "Set finish action" << std::endl;
    done = false;
    finishaction.sa_handler=finish;
    sigemptyset (&finishaction.sa_mask);
    finishaction.sa_flags=0;
    sigaction(SIGINT, &finishaction, nullptr);

    // Periodically check input queue.
    std::cout << "Reading MIDI from port " << midiin->getPortName() << " ... quit with Ctrl-C." << std::endl;
    while ( !done.load() ) {
      SLEEP( 10 );
    }
  } catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

 cleanup:
  delete midiin;
  return 0;
}
