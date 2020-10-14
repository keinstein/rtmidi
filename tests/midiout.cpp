//*****************************************//
//  midiout.cpp
//  by Gary Scavone, 2003-2004.
//
//  Simple program to test MIDI output.
//
//*****************************************//

#include "test-common.h"
#include "RtMidi.h"
#include <iostream>
#include <cstdlib>

int main( int /* argc*/, char */*argv*/[] )
{
  RtMidiOut *midiout = 0;
  std::vector<unsigned char> message;

  // RtMidiOut constructor
  try {
    midiout = new RtMidiOut();
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    exit( EXIT_FAILURE );
  }

  // Call function to select port.
  try {
    if ( chooseMidiPort( midiout ) == false ) goto cleanup;
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  // Send out a series of MIDI messages.

  // Program change: 192, 5
  message.push_back( 192 );
  message.push_back( 5 );
  try {
    midiout->sendMessage( &message );

    SLEEP( 500 );

    message[0] = 0xF1;
    message[1] = 60;
    midiout->sendMessage( &message );

    // Control Change: 176, 7, 100 (volume)
    message[0] = 176;
    message[1] = 7;
    message.push_back( 100 );
    midiout->sendMessage( &message );

    // Note On: 144, 64, 90
    message[0] = 144;
    message[1] = 64;
    message[2] = 90;
    midiout->sendMessage( &message );

    SLEEP( 500 );

    // Note Off: 128, 64, 40
    message[0] = 128;
    message[1] = 64;
    message[2] = 40;
    midiout->sendMessage( &message );

    SLEEP( 500 );

    // Control Change: 176, 7, 40
    message[0] = 176;
    message[1] = 7;
    message[2] = 40;
    midiout->sendMessage( &message );

    SLEEP( 500 );

    // Sysex: 240, 67, 4, 3, 2, 247
    message[0] = 240;
    message[1] = 67;
    message[2] = 4;
    message.push_back( 3 );
    message.push_back( 2 );
    message.push_back( 247 );
    midiout->sendMessage( &message );
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    goto cleanup;
  }

  // Clean up
 cleanup:
  std::cout << "midiout finished" << std::endl;
  delete midiout;

  return 0;
}
