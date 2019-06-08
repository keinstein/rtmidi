//*****************************************//
//  loopback
//  by Tobis Schlemmer, 2014.
//  inspired by virtual-loopback-test-automated.js from the node-midi project.
//  donated to RtMidi.
//
/*! \example loopback.cpp
  Simple program to test MIDI input and
  output in an internal loop using a user callback function.
*/
//*****************************************//

#include "RtMidi.h"
#include <iostream>
#include <cstdlib>
#include <cassert>

// Platform-dependent sleep routines.
#if defined(__WINDOWS_MM__)
#include <windows.h>
#define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds ) 
#else // Unix variants
#include <unistd.h>
#define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )
#endif


/* Here, we store the expected output. Timing is not tested */
std::vector<unsigned char> virtualinstring;
const char * virtualinstringgoal =
	"\xc0\x5\xf1\x3c\xb0\x7\x64\x90\x40\x5a\x80\x40\x28\xb0\x7\x28\xf0\x43\x4\x3\x2\xf7";
std::vector<unsigned char> instring;
const  char * instringgoal =
	"\xc0\x6\xf1\x3d\xb0\x8\x64\x90\x41\x5a\x80\x41\x28\xb0\x8\x28\xf0\x43\x4\x3\x3\xf7";

inline size_t getlength(const char * messages) {
	size_t retval = 0;
	const unsigned char * c = reinterpret_cast<const unsigned char *>(messages);
	while (*(c++) != 0xf7) retval++;
	return ++retval;
}

void mycallback1( double /* deltatime */, std::vector< unsigned char > *message, void * /* userData */ )
{
	unsigned int nBytes = message->size();
	for ( unsigned int i=0; i<nBytes; i++ ) {
		instring.push_back(message->at(i));
		//		std::cout << "\\x" << std::hex << (int)message->at(i) << std::flush;
	}
	/*	if ( nBytes > 0 )
		std::cout << "stamp = " << deltatime << std::endl;
	*/
}

void mycallback2( double /* deltatime */, std::vector< unsigned char > *message, void * /* userData */ )
{
	unsigned int nBytes = message->size();
	for ( unsigned int i=0; i<nBytes; i++ ) {
		virtualinstring.push_back(message->at(i));
		// std::cout << "\\x" << std::hex << (int)message->at(i);
	}
	/*
	  if ( nBytes > 0 )
	  std::cout << "stamp = " << deltatime << std::endl;
	*/
}

rtmidi::Pointer<rtmidi::PortDescriptor> getBrokenInputPortDescriptor(rtmidi::ApiType api = rtmidi::UNSPECIFIED) {
  // rtmidi::MidiIn constructor
  rtmidi::MidiIn virtualin(api,"Nonexisting input client");
  virtualin.openVirtualPort("RtMidi Test Virtual In");
  return virtualin.getDescriptor(true);
}

rtmidi::Pointer<rtmidi::PortDescriptor> getBrokenOutputPortDescriptor(rtmidi::ApiType api = rtmidi::UNSPECIFIED) {
  // rtmidi::MidiOut constructor
  rtmidi::MidiOut virtualout(api,"Nonexisting output client");;
  virtualout.openVirtualPort("RtMidi Test Virtual Out");
  return virtualout.getDescriptor(true);
}


int main( int /* argc */, char * /*argv*/[] )
{
  try {
    std::vector<rtmidi::ApiType> apis = rtmidi::Midi::getCompiledApi();
    for (auto api: apis) {

      std::cout << "Checking " << rtmidi::getApiName(api) << std::endl;
      rtmidi::Pointer<rtmidi::PortDescriptor> inputdescriptor =
	getBrokenInputPortDescriptor(api);

      rtmidi::Pointer<rtmidi::PortDescriptor> outputdescriptor =
	getBrokenOutputPortDescriptor(api);

      if (!inputdescriptor || !outputdescriptor) {
	assert(!(rtmidi::MidiIn(api).hasVirtualPorts()));
	assert(!(rtmidi::MidiOut(api).hasVirtualPorts()));
	assert(!inputdescriptor);
	assert(!outputdescriptor);
	std::cout << "API supports no virtual devices. Skipping test." << std::endl;
	continue;
      }

      assert(inputdescriptor);
      assert(outputdescriptor);

      sleep(2);

      rtmidi::Pointer<rtmidi::MidiInApi> ininapi(inputdescriptor->getInputApi());
      rtmidi::Pointer<rtmidi::MidiOutApi> inoutapi(inputdescriptor->getOutputApi());

      std::string inname = inputdescriptor->getName();
      int incapabilities = inputdescriptor->getCapabilities();

      std::cout << "Input: `" << inname << "` cap " << std::hex << incapabilities << std::endl;

      if (ininapi) {
	ininapi->openPort(outputdescriptor);
      }
      if (inoutapi) {
	inoutapi->openPort(inputdescriptor);
      }

      rtmidi::Pointer<rtmidi::MidiInApi> outinapi(outputdescriptor->getInputApi());
      rtmidi::Pointer<rtmidi::MidiOutApi> outoutapi(outputdescriptor->getOutputApi());

      std::string outname = outputdescriptor->getName();
      int outcapabilities = outputdescriptor->getCapabilities();

      std::cout << "Output: `" << outname << "` cap " << std::hex << outcapabilities << std::endl;

      if (outinapi) {
	outinapi->openPort(outputdescriptor);
      }
      if (outoutapi) {
	outoutapi->openPort(inputdescriptor);
      }

    }
  } catch ( rtmidi::Error &error ) {
    error.printMessage();
  }
  return 0;
}
