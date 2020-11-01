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
#include <signal.h>

std::atomic<bool> done;
static void finish( int /*ignore*/ ){
  done = true;
}
struct sigaction finishaction;

void usage( void ) {
  // Error function in case of incorrect command-line
  // argument specifications.
  std::cout << "\nuseage: loopback-device <clientname>\n";
  std::cout << "    clientname = name of the clientname.\n\n";
  exit( 0 );
}

std::string clientname = "RtMidi Loopback device";
rtmidi::ApiType apitype = rtmidi::UNSPECIFIED;

               
class LoopbackInterface: public rtmidi::SplitSysexMidiInterface {
public:
  LoopbackInterface(rtmidi::Midi & m):midi(m) {}
  virtual ~LoopbackInterface ( ) {}

  //! The MIDI callback function.
  /*! This function is called whenever a MIDI packet is received by a
    MIDI backend that uses the corresponding callback object.
    \param timestamp the timestamp when the MIDI message has been received
    \param message the message itself.
  */
  virtual void midiIn ( double timestamp, std::vector<unsigned char>& message ) throw() {
    midi.sendMessage( message.data(), message.size() );
  }

  virtual void midiIn ( double timestamp, unsigned char * begin, ptrdiff_t size) throw() {
    midi.sendMessage( begin, size );
  }

  //! Receive a chunk of a system exclusive message.
  /**
   * System exclusive messages are delviered in chunks in some
   * setups. The first chunk always start with 0xf0, while the last
   * one ends with 0xf7. All other chunks are considered to be
   * intermediate chunks.
   *
   * \param timestamp Time when the chunk was received.
   * \param begin Pointer to the first character of the chunk.
   * \param size Length of the chunk in characters.
   * \param connectionId Id of the sending interface. As some
   * interfaces allow several connections to a certain port, this variable can help to
   * deal with interleaving packets.
   */
  virtual void sysexChunk ( double timestamp,
                            unsigned char * begin,
                            ptrdiff_t size,
                            int connectionId ) throw() {
    midiIn ( timestamp, begin, size );
  }

  //! Delete the object if necessary.
  /*! This function allows the user to delete the Midi callback object,
    when MIDI backend drops its reference to it. By default it does nothing.
    But, callback objects are owned by the MIDI backend. These must be deleted
    after the reference to them has been dropped.

    \sa CompatibilityMidiInterface
  */
  virtual void deleteMe ( ) { delete this; }
protected:
  rtmidi::Midi &midi;
};

int main( int argc, char *argv[] )
{
  if ( argc > 1 )
    clientname = argv[1];
  if ( argc > 2 ) {
    for (int i = rtmidi::UNSPECIFIED;
         i != rtmidi::NUM_APIS;
         ++i) {
      // namen Filtern
    }
  }
    
  try {
    rtmidi::Midi midi(apitype,
                      rtmidi::PortDescriptor::INOUTPUT,
                      clientname);

    midi.openVirtualPort( "Port",
                          rtmidi::PortDescriptor::INOUTPUT);

    midi.setCallback(new LoopbackInterface(midi));
  
    // Don't ignore sysex, timing, or active sensing messages.
    midi.ignoreTypes( false, false, false );


    // Install an interrupt handler function.
    std::cout << "Set finish action" << std::endl;
    done = false;
    finishaction.sa_handler=finish;
    sigemptyset (&finishaction.sa_mask);
    finishaction.sa_flags=0;
    sigaction(SIGINT, &finishaction, nullptr);

    // Periodically check input queue.
    std::cout << "Reading MIDI from port " << midi.getDescriptor(true)->getName() << " ... quit with Ctrl-C." << std::endl;
    while ( !done.load() ) {
      SLEEP( 10 );
    }
  } catch ( RtMidiError &error ) {
    error.printMessage();
    exit(1);
  }
  return 0;
}
