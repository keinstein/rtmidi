//*****************************************//
//  cmidiin.cpp
//  by Gary Scavone, 2003-2014.
//  and Tobias Schlemmer 2014.
//
/*! \example cmidiin2.cpp
  Simple program to test MIDI input and
  use of a user callback function.
*/
//
//*****************************************//

#include "test-common.h"
#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <signal.h>

std::atomic_bool done;
static void finish( int /*ignore*/ )
{
  done = true;
}
struct sigaction finishaction;


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

  std::vector<unsigned char> message;
#if 0
  std::cout << "\nWould you like to check all output ports? [Y/n] ";

  std::string keyHit;
  std::getline( std::cin, keyHit );
  if ( keyHit == "n" ) {
    type = rtmidi::UNSPECIFIED;
  }
#endif
  rtmidi::ApiType type = rtmidi::ALL_API;

  try {

    // rtmidi::MidiIn constructor
    rtmidi::MidiIn midiin(type);

    // Set our callback function.  This should be done immediately after
    // opening the port to avoid having incoming messages written to the
    // queue instead of sent to the callback function.
    midiin.setCallback( &mycallback );

    rtmidi::PortList list = midiin.getPortList(rtmidi::PortDescriptor::INPUT);


    // Minimal command-line check.
    if ( argc > 2 ) usage("cmidin2", list, rtmidi::PortDescriptor::INPUT);

    try {
      rtmidi::Pointer<rtmidi::PortDescriptor> port = 0;
      // Check available ports vs. specified.
      if ( ! ( argc == 2 ?
               openMidiPort( argv[1], midiin, list, rtmidi::PortDescriptor::INPUT ) :
               chooseMidiPort( midiin, list, rtmidi::PortDescriptor::INPUT )
               )
           ) {
      }

    }
    catch ( rtmidi::Error &error ) {
      error.printMessage();
      return 1;
    }

    // Don't ignore sysex, timing, or active sensing messages.
    midiin.ignoreTypes( false, false, false );

    // Install an interrupt handler function.
    std::cout << "Set finish action" << std::endl;
    done = false;
    finishaction.sa_handler=finish;
    sigemptyset (&finishaction.sa_mask);
    finishaction.sa_flags=0;
    sigaction(SIGINT, &finishaction, nullptr);


    // Periodically check input queue.
    std::cout << "Reading MIDI from port ... " << midiin.getPortName()
              << " quit with Ctrl-C." << std::endl;
    while ( !done ) {
      // Sleep for 50 milliseconds.
      SLEEP( 50 );
    }
  }
  catch ( rtmidi::Error &error ) {
    error.printMessage();
    exit( EXIT_FAILURE );
  }

  return 0;
}
