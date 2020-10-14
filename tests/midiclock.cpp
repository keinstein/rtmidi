//*****************************************//
//  midiclock.cpp
//
//  Simple program to test MIDI clock sync.  Run midiclock_in in one
//  console and midiclock_out in the other, make sure to choose
//  options that connect the clocks between programs on your platform.
//
//  (C)2016 Refer to README.md in this archive for copyright.
//
//*****************************************//

#include "test-common.h"
#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <cmath>

std::atomic<bool> done;
static void finish( int /*ignore*/ ){
  done = true;
}
struct sigaction finishaction;


bool verbose;

void mycallback( double deltatime, std::vector< unsigned char > *message, void *user )
{
  unsigned int *clock_count = reinterpret_cast<unsigned int*>(user);

  // Ignore longer messages
  if (message->size() != 1)
    return;

  unsigned int msg = message->at(0);
  if (msg == 0xFA)
    std::cout << "START received" << std::endl;
  if (msg == 0xFB)
    std::cout << "CONTINUE received" << std::endl;
  if (msg == 0xFC)
    std::cout << "STOP received" << std::endl;
  if (msg == 0xF8) {
    if (++*clock_count == 24) {
      double bpm = 60.0 / 24.0 / deltatime;
      if (!verbose)
        bpm = std::round(bpm/20)*20;
      printf(verbose?"One beat, estimated BPM = %.2f":"One beat, estimated BPM = %.0f\n",
             bpm);
      *clock_count = 0;
    }
  }
  else
    *clock_count = 0;
}

int clock_in()
{
  RtMidiIn *midiin = 0;
  unsigned int clock_count = 0;

  try {

    // RtMidiIn constructor
    midiin = new RtMidiIn();

    chooseMidiPort(midiin);

    // Set our callback function.  This should be done immediately after
    // opening the port to avoid having incoming messages written to the
    // queue instead of sent to the callback function.
    midiin->setCallback( &mycallback, &clock_count );

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
  }

 cleanup:

  delete midiin;

  return 0;
}

int clock_out()
{
  RtMidiOut *midiout = 0;
  std::vector<unsigned char> message;
  int sleep_ms = 0, k = 0, j = 0;

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
    chooseMidiPort(midiout);
  }
  catch ( RtMidiError &error ) {
    error.printMessage();
    goto cleanup;
  }

  // Period in ms = 100 BPM
  // 100*24 ticks / 1 minute, so (60*1000) / (100*24) = 25 ms / tick
  sleep_ms = 25;
  std::cout << "Generating clock at "
            << (60.0 / 24.0 / sleep_ms * 1000.0)
            << " BPM." << std::endl;

  // Send out a series of MIDI clock messages.
  // MIDI start
  message.clear();
  message.push_back( 0xFA );
  try {
    midiout->sendMessage( &message );
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    delete midiout;
    return -1;
  }
  
  std::cout << "MIDI start" << std::endl;

  try {
    for (j=0; j < 8; j++)
      {
	if (j > 0)
	  {
	    // MIDI continue
	    message.clear();
	    message.push_back( 0xFB );
	    midiout->sendMessage( &message );
	    std::cout << "MIDI continue" << std::endl;
	  }

	for (k=0; k < 96; k++) {
	  // MIDI clock
	  message.clear();
	  message.push_back( 0xF8 );
	  midiout->sendMessage( &message );
	  if (k % 24 == 0)
	    std::cout << "MIDI clock (one beat)" << std::endl;
	  SLEEP( sleep_ms );
	}

	// MIDI stop
	message.clear();
	message.push_back( 0xFC );
	midiout->sendMessage( &message );
	std::cout << "MIDI stop" << std::endl;
	SLEEP( 500 );
      }
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    delete midiout;
    return -1;
  }


  // MIDI stop
  message.clear();
  message.push_back( 0xFC );
  try {
    midiout->sendMessage( &message );
  } catch (const rtmidi::Error & e) {
    e.printMessage();
    delete midiout;
    return -1;
  }
  std::cout << "MIDI stop" << std::endl;

  SLEEP( 500 );

  std::cout << "Done!" << std::endl;

  // Clean up
 cleanup:
  delete midiout;

  return 0;
}

int main( int argc, const char *argv[] )
{
  std::string prog(argv[0]);
  verbose = argc > 1;
  if (prog.find("midiclock_in") != prog.npos) {
    clock_in();
  }
  else if (prog.find("midiclock_out") != prog.npos) {
    clock_out();
  }
  else {
    std::cout << "Don't know what to do as " << prog << std::endl;
  }
  return 0;
}

