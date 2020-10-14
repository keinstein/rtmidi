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

int main( int argc, char *argv[] )
{
  std::vector<unsigned char> message;
  int nBytes, i;
  double stamp;


  // rtmidi::MidiIn constructor
  try {
    rtmidi::MidiQueueInterface queue(256);
    rtmidi::MidiIn midiin(rtmidi::ALL_API);
    midiin.setCallback(&queue);


    rtmidi::PortList list = midiin.getPortList(rtmidi::PortDescriptor::INPUT);


    // Minimal command-line check.
    if ( argc > 2 ) usage("qmidin2", list, rtmidi::PortDescriptor::INPUT);

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
      while ( !queue.empty() ) {
        stamp = queue.getMessage( message );
        nBytes = message.size();
        for ( i=0; i<nBytes; i++ )
          std::cout << "Byte " << i << " = " << (int)message[i] << ", ";
        if ( nBytes > 0 )
          printf("stamp = %.2f\n", stamp);
      }

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
