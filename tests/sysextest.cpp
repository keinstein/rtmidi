//*****************************************//
//  by Gary Scavone, 2003-2005.
/*!  \example sysextest.cpp

  program to test MIDI sysex sending and receiving.
*/
//*****************************************//

#include "test-common.h"
#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <typeinfo>

void usage( void ) {
  std::cout << "\nuseage: sysextest N\n";
  std::cout << "    where N = length of sysex message to send / receive.\n\n";
}


void mycallback( double deltatime, std::vector< unsigned char > *message, void * /*userData*/ )
{
  unsigned int nBytes = message->size();
  for ( unsigned int i=0; i<nBytes; i++ )
    std::cout << "Byte " << i << " = " << (int)message->at(i) << ", ";
  if ( nBytes > 0 )
    std::cout << "stamp = " << deltatime << std::endl;
}

int main( int argc, char *argv[] )
{
  RtMidiOut *midiout = 0;
  RtMidiIn *midiin = 0;
  std::vector<unsigned char> message;
  unsigned int i, nBytes;

  usage();

  // Minimal command-line check.
  if ( argc != 2 ) nBytes = 100000;
  else
    nBytes = (unsigned int) atoi( argv[1] );

  // RtMidiOut and RtMidiIn constructors
  try {
    midiout = new RtMidiOut();
    midiin = new RtMidiIn();
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  // Don't ignore sysex, timing, or active sensing messages.
  midiin->ignoreTypes( false, true, true );

  try {
    if ( chooseMidiPort( midiin  ) == false ) goto cleanup;
    if ( chooseMidiPort( midiout ) == false ) goto cleanup;
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  try {
    midiin->setCallback( &mycallback );
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  message.push_back( 0xF6 );
  try {
    midiout->sendMessage( &message );
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    goto cleanup;
  }
  SLEEP( 500 ); // pause a little

  try {
    // Create a long sysex message of numbered bytes and send it out ... twice.
    for ( int n=0; n<2; n++ ) {
      message.clear();
      if (nBytes >0) {
	message.reserve(nBytes);
      }
      message.push_back( 240 );
      for ( i=0; i<nBytes; i++ )
	message.push_back( i % 128 );
      message.push_back( 247 );
      midiout->sendMessage( &message );

      SLEEP( 500 ); // pause a little
    }
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    goto cleanup;
  }

  // Clean up
 cleanup:
  delete midiout;
  delete midiin;

  return 0;
}
