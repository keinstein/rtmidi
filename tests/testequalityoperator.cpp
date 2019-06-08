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


rtmidi::PortPointer getOutputPort(rtmidi::ApiType api) {
  rtmidi::MidiOut out(api,"Output client");
  rtmidi::PortList list = out.getPortList();
  if (list.empty())
    return rtmidi::PortPointer(0);
  return list.front();
}

rtmidi::PortPointer getInputPort(rtmidi::ApiType api) {
  rtmidi::MidiIn in(api,"Input client");
  rtmidi::PortList list = in.getPortList();
  if (list.empty())
    return rtmidi::PortPointer(0);
  return list.front();
}

int main( int /* argc */, char * /*argv*/[] )
{
  try {
    std::vector<rtmidi::ApiType> apis = rtmidi::Midi::getCompiledApi();
    for (auto api: apis) {

      rtmidi::MidiIn virtualin(api,"Input client");
      if (virtualin.hasVirtualPorts())
	virtualin.openVirtualPort("RtMidi Test Virtual In");

      rtmidi::MidiOut virtualout(api,"Nonexisting output client");
      if (virtualout.hasVirtualPorts())
	virtualout.openVirtualPort("RtMidi Test Virtual Out");

      rtmidi::Pointer<rtmidi::PortDescriptor>
	inputdescriptor = virtualin.hasVirtualPorts() ? virtualin.getDescriptor(true):getOutputPort(api),
	outputdescriptor = virtualout.hasVirtualPorts() ? virtualout.getDescriptor(true):getInputPort(api);

      rtmidi::PortList inputPortList = virtualin.getPortList();
      rtmidi::PortList outputPortList = virtualout.getPortList();

      if (!inputPortList.empty()) {
	int count = 0;
	for (rtmidi::PortPointer & p: inputPortList) {
	  if (*p == *outputdescriptor) count++;
	}
	std::cout << "found " << count << " output ports." << std::endl;
	assert(count == 1);
      }

      if (!outputPortList.empty()) {
	int count = 0;
	for (rtmidi::PortPointer & p: outputPortList) {
	  if (*p == *inputdescriptor) count++;
	}
	std::cout << "found " << count << " input ports." << std::endl;
	assert(count == 1);
      }
    }
  } catch ( rtmidi::Error &error ) {
    error.printMessage();
  }
  return 0;
}
