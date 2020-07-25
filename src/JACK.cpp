/**********************************************************************/
/*! \file
  \brief JACK driver for RtMidi

  This file contains the jack driver class JackMidi and supporting
  material.

  RtMidi WWW site: http://music.mcgill.ca/~gary/rtmidi/

  Midi: realtime MIDI i/o C++ classes
  Copyright (c) 2003-2017 Gary P. Scavone
  Forked by Tobias Schlemmer, 2014-2018.

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation files
  (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge,
  publish, distribute, sublicense, and/or sell copies of the Software,
  and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  Any person wishing to distribute modifications to the Software is
  asked to send the modifications to the original developer so that
  they can be incorporated into the canonical version. This is,
  however, not a binding provision of this license.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
  ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/**********************************************************************/

#include "RtMidi.h"
#include "src/RtMidi-internal.h"
#include "src/JACK.h"
#include <sstream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <functional>
#include <cerrno>
#ifndef RTMIDI_FALLTHROUGH
#define RTMIDI_FALLTHROUGH
#endif


// Default for Windows is to add an identifier to the port names; this
// flag can be defined (e.g. in your project file) to disable this behaviour.
//#define RTMIDI_DO_NOT_ENSURE_UNIQUE_PORTNAMES


#if !defined( __LINUX_ALSA__ ) && !defined( __UNIX_JACK__ ) && !defined( __MACOSX_COREMIDI__ ) && !defined( __WINDOWS_MM__ )
#define __RTMIDI_DUMMY__
#endif

#ifndef N_
#define N_( x ) x
#endif

// **************************************************************** //
//
// Common helpers.
//
// **************************************************************** //


RTMIDI_NAMESPACE_START

//*********************************************************************//
// API: UNIX JACK
//
// Written primarily by Alexander Svetalkin, with updates for delta
// time by Gary Scavone, April 2011.
//
// *********************************************************************//

#if defined( __UNIX_JACK__ )

RTMIDI_NAMESPACE_END
// JACK header files
#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/ringbuffer.h>
#ifdef HAVE_SEMAPHORE
#include <semaphore.h>
#endif
RTMIDI_NAMESPACE_START

#define JACK_RINGBUFFER_SIZE 16384 // Default size for ringbuffer



std::shared_ptr<LockingJackSequencer> JackPortDescriptor :: main_seq;


#define RTMIDI_CLASSNAME "JackPortDescriptor"
PortList JackPortDescriptor :: getPortList( int capabilities,
                                            std::shared_ptr<LockingJackSequencer> & s)
{
  PortList list;
  const char ** ports = s->getPortList( jackCapabilities( capabilities ) );
  if ( !ports ) return list;
  for ( const char ** port = ports; *port; port++ ) {
    list.push_back( Pointer<PortDescriptor>(new JackPortDescriptor( *port, s ) ) );
  }
  jack_free( ports );
  return list;
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "JackSequencer"

template <int locking>
int JackSequencer<locking> :: Callback( jack_nframes_t nframes, void * arg )
{
  JackSequencer * self = reinterpret_cast<JackSequencer *>(arg);
  if (!self) return 1;
  for (ApiPortList::LockedItemIterator i(self->ports);
       i != self->ports.end();
       ++i) {

    JackMidi * data = i->port;
    MidiJack * rtData = data->api;
    jack_midi_event_t event;
    jack_time_t time;
    double timeStamp;
    size_t space;
    jack_midi_data_t * midiData;

    // Is port created?
    if ( data->local == NULL ) {
      data->deletePortIfRequested( );
      return 0;
    }
    void * buff = jack_port_get_buffer( data->local, nframes );

    if ( buff != NULL ) {
      // handle MIDI input

      // We have midi events in buffer
      int evCount = jack_midi_get_event_count( buff );
      for ( int j = 0; j < evCount; j++ ) {
        jack_midi_event_get( &event, buff, j );

        // Compute the delta time.
        time = jack_get_time( );
        if ( rtData->firstMessage == true ) {
          timeStamp = 0.0;
          rtData->firstMessage = false;
        } else
          timeStamp = ( time - data->lastTime ) * 0.000001;

        data->lastTime = time;

        if ( !rtData->continueSysex && event.size) {
          rtData->doMidiCallback( timeStamp, event.buffer, event.size );
        }
      }

      // handle MIDI output
      if (data->buffSize) {
        jack_midi_clear_buffer( buff );

        while ( jack_ringbuffer_read_space( data->buffSize ) > 0 ) {
          jack_ringbuffer_read( data->buffSize,
                                reinterpret_cast<char *>(&space),
                                sizeof( space ) );
          midiData = jack_midi_event_reserve( buff, 0, space );

          jack_ringbuffer_read( data->buffMessage,
                                reinterpret_cast<char *>(midiData),
                                space );
        }
      }
    }

    data->deletePortIfRequested( );
  }
  return 0;
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "JackPortDescriptor"
MidiApi * JackPortDescriptor :: getApi ( unsigned int capabilities ) const {
  // either input or output or both can be handled by the same API class
  if ( capabilities & INOUTPUT ) {
    MidiApi * api = new MidiJack( "" );
#if 0
    if (api) api->setCurrentCapabilities( capabilities );
#endif
    return api;
  } else
    return nullptr;
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "MidiJack"
MidiJack :: MidiJack( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiJack::initialize( clientName );
}


#if 0
void MidiJack :: connect( )
{
  abort( );
  // this should be unnecessary
  JackMidi * data = static_cast<JackMidi *> ( apiData_ );
  if ( data->local )
    return;

  // Initialize JACK client
  if ( ( data->local = jack_client_open( clientName.c_str( ), JackNoStartServer, NULL ) ) == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "JACK server not running?" ),
                         Error::WARNING ) );
    return;
  }

  jack_set_process_callback( data->client, ProcessIn, data );
  jack_activate( data->client );
}
#endif

MidiJack :: ~MidiJack( )
{
  //  closePort( );
  try {
    if ( midi )
      midi -> request_delete( );
  } catch ( const Error& e ) {
    e.printMessage( std::cerr );
  }
}

void MidiJack :: openVirtualPort( const std::string& portName,
                                  unsigned int capabilities)
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  try {
    unsigned long jackCapabilities =
      ((capabilities & PortDescriptor::OUTPUT) ? JackPortIsOutput : 0)
      | ((capabilities & PortDescriptor::INPUT) ? JackPortIsInput : 0);
    midi->ensureOpen( jackCapabilities,
                      portName );
    connected_ = true;
  } catch ( Error& e ) {
    error ( e );
  }
}

size_t MidiJack :: maxSysExSize() const {
  if ( !midi ) {
    const_cast<MidiJack *>(this)->error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return -1;
  }
  return midi->maxSysExSize();
}


void MidiJack :: openPort( const PortDescriptor& p,
                           const std::string& portName,
                           unsigned int capabilities)
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  const JackPortDescriptor * port = dynamic_cast<const JackPortDescriptor *>( &p );

  if ( !port ) {
    error( RTMIDI_ERROR( gettext_noopt( "JACK has been instructed to open a non-JACK MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  openVirtualPort( portName, capabilities );
  try {
    if (capabilities  & PortDescriptor::INPUT)
      midi->connectFrom( *port );
    if (capabilities  & PortDescriptor::OUTPUT)
      midi->connectTo( *port );
  } catch ( Error& e ) {
    closePort( );
    error ( e );
  }
}

Pointer<PortDescriptor> MidiJack :: getDescriptor( bool isLocal )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return nullptr;
  }
  try {
    return midi->getDescriptor( isLocal );
  } catch ( Error& e ) {
    error( e );
  }
  return nullptr;
}

PortList MidiJack :: getPortList( int capabilities )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return PortList( );
  }
  try {
    return midi->getPortList(capabilities);
  } catch ( Error& e ) {
    error( e );
  }
  return PortList( );
}

void MidiJack :: closePort( )
{
#if defined( __RTMIDI_DEBUG__ )
  std::cerr << "Closing Port" << std::endl;
#endif
  if ( midi )
    midi->delayedDeletePort( );
  connected_ = false;
}

void MidiJack :: setClientName( const std::string& name )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  midi->setClientName( name );
}

void MidiJack :: setPortName( const std::string& portName )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  midi->rename( portName );
}


void MidiJack :: initialize( const std::string& clientName )
{
  midi = new JackMidi( this, clientName );
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Cannot allocate JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  // init is the last as it may throw an exception
  try {
    midi->init( );
    if (!clientName.empty()) {
      setClientName( clientName );
    }
  } catch ( const Error& e ) {
    delete midi;
    midi = nullptr;
    error( e );
  }
}
#undef RTMIDI_CLASSNAME
//*********************************************************************//
// API: JACK
// Class Definitions: MidiJack
//*********************************************************************//


#define RTMIDI_CLASSNAME "MidiJack"
#if 0
unsigned int MidiJack :: getPortCount( )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return 0;
  }
  return midi->getPortCount( JackPortIsInput );
}
std::string MidiJack :: getPortName( unsigned int portNumber )
{
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Missing JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return"";
  }
  return midi->getPortName( portNumber,
                            JackPortIsInput );
}
#endif

void MidiJack :: sendMessage( const unsigned char * message, size_t size )
{
  if ( !midi ) return;

  // Write full message to buffer
  jack_ringbuffer_write( midi->buffMessage,
                         reinterpret_cast<const char *>( message ),
                         size );
  jack_ringbuffer_write( midi->buffSize,
                         reinterpret_cast<char *>( &size ),
                         sizeof( size ) );
}
#undef RTMIDI_CLASSNAME
#endif // __UNIX_JACK__

RTMIDI_NAMESPACE_END
