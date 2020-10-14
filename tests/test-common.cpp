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


bool chooseMidiPort( RtMidi *rtmidi )
{
  bool isInput = false;
  if ( typeid( *rtmidi ) == typeid( RtMidiIn ) )
    isInput = true;

  if ( isInput )
    std::cout << "\nWould you like to open a virtual input port? [y/N] ";
  else
    std::cout << "\nWould you like to open a virtual output port? [y/N] ";

  std::string keyHit;
  std::getline( std::cin, keyHit );
  if ( keyHit == "y" ) {
    rtmidi->openVirtualPort();
    return true;
  }

  std::string portName;
  unsigned int i = 0, nPorts = rtmidi->getPortCount();
  if ( nPorts == 0 ) {
    if ( isInput )
      std::cout << "No input ports available!" << std::endl;
    else
      std::cout << "No output ports available!" << std::endl;
    return false;
  }

#if 0
  if ( nPorts == 1 ) {
    std::cout << "\nOpening " << rtmidi->getPortName() << std::endl;
  }
  else {
#endif
    for ( i=0; i<nPorts; i++ ) {
      portName = rtmidi->getPortName(i);
      if ( isInput )
        std::cout << "  Input port #" << i << ": " << portName << '\n';
      else
        std::cout << "  Output port #" << i << ": " << portName << '\n';
    }

    do {
      std::cout << "\nChoose a port number: ";
      std::cin >> i;
    } while ( i >= nPorts );
#if 0
  }
#endif

  std::cout << std::endl;
  rtmidi->openPort( i );

  return true;
}

static const int usageDisplayFlags
= ( rtmidi::PortDescriptor::SESSION_PATH |
    rtmidi::PortDescriptor::UNIQUE_PORT_NAME |
    rtmidi::PortDescriptor::INCLUDE_API );


void usage ( const char * name,
             rtmidi::PortList & list,
             rtmidi::PortDescriptor::PortCapabilities flags)
{
  (void) flags;
  std::cout << "\nusage: " << name << " \"<port>\"\n";
  std::cout << "    where port = the device to use (default = first available port).\n\n";

  std::cout << "Available ports:" << std::endl;
  for (rtmidi::PortList::iterator i = list.begin();
       i != list.end(); i++) {
    std::cout << "\""
	      << (*i)->getName( usageDisplayFlags )
	      << "\"";
    std::cout << "\t";
    std::cout << (*i)->getName() << std::endl;
  }
  exit( 0 );
}

bool openMidiPort( const char * name,
                   rtmidi::Midi & midi,
                   rtmidi::PortList list,
                   rtmidi::PortDescriptor::PortCapabilities flags )
{
  for (rtmidi::PortList::iterator i = list.begin();
       i != list.end(); i++) {
    if ( name == (*i)->getName( usageDisplayFlags ) ) {
      midi.openPort( *i, "RtMidi test port", flags );
      return true;
    }
  }
  std::cout << "Invalid port specifier!\n";
  return false;
}

bool chooseMidiPort( rtmidi::Midi & midi,
                     rtmidi::PortList list,
                     rtmidi::PortDescriptor::PortCapabilities flags) {
  switch (flags) {
  case rtmidi::PortDescriptor::INPUT:
    std::cout << "\nWould you like to open a virtual input port? [y/N] ";
    break;
  case rtmidi::PortDescriptor::OUTPUT:
    std::cout << "\nWould you like to open a virtual output port? [y/N] ";
    break;
  default:
    return false;
  }

  std::string keyHit;
  std::getline( std::cin, keyHit );
  if ( keyHit == "y" ) {
    midi.openVirtualPort( "RtMidi test port", flags );
    return true;
  }

  std::string portName;
  unsigned int i = 0, nPorts = !list.empty();
  if ( nPorts == 0 ) {
    switch (flags) {
    case rtmidi::PortDescriptor::INPUT:
      std::cout << "No input ports available!" << std::endl;
      break;
    case rtmidi::PortDescriptor::OUTPUT:
      std::cout << "No output ports available!" << std::endl;
      break;
    case rtmidi::PortDescriptor::INOUTPUT:
    case rtmidi::PortDescriptor::VIRTUAL:
    case rtmidi::PortDescriptor::UNLIMITED:
      break;
    }
    return false;
  }

  nPorts = 0;
  for ( auto & port : list ) {
    switch (flags) {
    case rtmidi::PortDescriptor::INPUT:
      std::cout << "  Input port #"  << nPorts << ": " << port->getName( ) << '\n';
      break;
    case rtmidi::PortDescriptor::OUTPUT:
      std::cout << "  Output port #" << nPorts << ": " << port->getName( ) << '\n';
      break;
    case rtmidi::PortDescriptor::INOUTPUT:
    case rtmidi::PortDescriptor::VIRTUAL:
    case rtmidi::PortDescriptor::UNLIMITED:
      break;
    }
    ++nPorts;
  }

  do {
    std::cout << "\nChoose a port number: ";
    std::cin >> i;
  } while ( i >= nPorts );
  std::cout << std::endl;

  auto port = list.begin();
  for (;i; --i) ++port;

  midi.openPort( *port, "RtMidi test port", flags );
  return true;
}
