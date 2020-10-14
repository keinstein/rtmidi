/**********************************************************************/
/*! \class Midi
  \brief An abstract base class for realtime MIDI input/output.

  This class implements some common functionality for the realtime
  MIDI input/output subclasses MidiIn and MidiOut.

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

struct  InternalMidiApi: public MidiApi {
  InternalMidiApi()
    : MidiApi( ),
      firstMessage( true ),
      continueSysex( false ),
      userCallback(nullptr){}

  ~InternalMidiApi( void )
  {
    if ( userCallback )
      userCallback->deleteMe( );
    // Delete the MIDI queue.
    //if ( queue.ringSize > 0 ) delete [] queue.ring;
  }

  void setCallback( SplitSysexMidiInterface * callback );
  void cancelCallback( );
  bool doMidiCallback( double timestamp, unsigned char * buffer, ptrdiff_t size) {
    if (userCallback) {
      userCallback->midiIn(timestamp, buffer, size);
      return true;
    }
    return false;
  }
#if 0
  double getMessage( std::vector<unsigned char>& message );
#endif

  void ignoreTypes( bool midiSysex, bool midiTime, bool midiSense );

  using MidiApi::openVirtualPort;
  using MidiApi::openPort;
  void openPort( unsigned int portNumber,
                 const std::string& portName,
                 unsigned int capabilities);

  bool firstMessage;
  bool continueSysex;
protected:
  char ignoreFlags;
  MidiInterface * userCallback;
};

void InternalMidiApi :: openPort( unsigned int portNumber,
                                  const std::string& portName,
                                  unsigned int capabilities)
{
  PortList list = getPortList(capabilities);
  unsigned int i = 0;
  for ( auto  & port : list ) {
    if (i++ == portNumber) {
      openPort(port, portName, capabilities);
    }

  }
}


// Define API names and display names.
// Must be in same order as API enum.
extern "C" {
  struct api_name {
    ApiType Id;
    const char * machine;
    const char * display;
  };
  struct api_name rtmidi_api_names[] =
    {
     { UNSPECIFIED, "unspecified" , N_( "Automatic backend selection" ) },
     { MACOSX_CORE, "core" , N_( "Core MIDI" ) },
     { LINUX_ALSA, "alsa" , N_( "ALSA" ) },
     { UNIX_JACK, "jack" , N_( "JACK" ) },
     { WINDOWS_MM, "winmm" , N_( "Windows Multimedia" ) },
     { WINDOWS_KS, "winks" , N_( "DirectX/Kernel Streaming" ) },
     { DUMMY, "dummy" , N_( "Dummy/NULL device" ) },
     { ALL_API, "allapi" , N_( "All available MIDI systems" ) },
    };
  const unsigned int rtmidi_num_api_names =
    sizeof( rtmidi_api_names )/sizeof( rtmidi_api_names[0] );


  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const ApiType rtmidi_compiled_system_apis[] =
    {
#if defined( __MACOSX_CORE__ )
     MACOSX_CORE,
#endif
#if defined( __LINUX_ALSA__ )
     LINUX_ALSA,
#endif
#if defined( __WINDOWS_MM__ )
     WINDOWS_MM,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_system_apis =
    sizeof( rtmidi_compiled_system_apis )/sizeof( rtmidi_compiled_system_apis[0] );

  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const ApiType rtmidi_compiled_software_apis[] =
    {
#if defined( __UNIX_JACK__ )
     UNIX_JACK,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_software_apis =
    sizeof( rtmidi_compiled_software_apis )/sizeof( rtmidi_compiled_software_apis[0] );


  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const ApiType rtmidi_compiled_other_apis[] =
    {
     UNSPECIFIED,
     ALL_API,
#if defined( __RTMIDI_DUMMY__ )
     DUMMY,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_other_apis =
    sizeof( rtmidi_compiled_other_apis )/sizeof( rtmidi_compiled_other_apis[0] );
}

// Flags for receiving MIDI
enum {
      IGNORE_SYSEX = 0x01,
      IGNORE_TIME = 0x02,
      IGNORE_SENSING = 0x04
} ignore_flags;

#if !defined( __LINUX_ALSA__ ) && !defined( __UNIX_JACK__ ) && !defined( __MACOSX_COREMIDI__ ) && defined( __WINDOWS_MM__ )
struct pthread_mutex_t;
constexpr inline int pthread_mutex_lock( pthread_mutex_t* ) { return 0; }
constexpr inline int pthread_mutex_unlock( pthread_mutex_t* ) { return 0; }
#endif

#define RTMIDI_CLASSNAME "scoped_lock"
template<bool locking>
struct scoped_lock {
  pthread_mutex_t * mutex;
  scoped_lock( pthread_mutex_t& m )
    : mutex( &m )
  {
    int retval = 0;
    // pthread_mutex_lock may report errors.
    if ( locking )
      while ( ( retval = pthread_mutex_lock( mutex ) ) == EINTR );
    if ( retval ) {
      RTMIDI_ERROR( gettext_noopt( "Internal error: Could not lock the mutex." ),
                    Error::WARNING );
    }
  }
  ~scoped_lock( )
  {
    if ( locking )
      while ( pthread_mutex_unlock( mutex ) == EINTR );
  }
};
#undef RTMIDI_CLASSNAME

// trim from start
static inline std::string& ltrim( std::string& s ) {
  s.erase( s.begin( ),
           std::find_if( s.begin( ), s.end( ), []( int ch ) {
                                                 return ! std::isspace( ch );
                                               } ) );
  return s;
}

// trim from end
static inline std::string& rtrim( std::string& s ) {
  s.erase( std::find_if( s.rbegin( ), s.rend( ), []( int ch ) {
                                                   return ! std::isspace( ch );
                                                 } ).base( ), s.end( ) );
  return s;
}

// trim from both ends
static inline std::string& trim( std::string& s ) {
  return ltrim( rtrim( s ) );
}

// **************************************************************** //
//
// InternalMidiApi and InternalMidiApi subclass prototypes.
//
// **************************************************************** //


#if defined( __MACOSX_COREMIDI__ )
RTMIDI_NAMESPACE_END
#if TARGET_OS_IPHONE
#define AudioGetCurrentHostTime CAHostTimeBase::GetCurrentTime
#define AudioConvertHostTimeToNanos CAHostTimeBase::ConvertToNanos
#endif
struct MIDIPacketList;
RTMIDI_NAMESPACE_START



class MidiInCore : public InternalMidiApi
{
public:
  MidiInCore( const std::string& clientName );
  ~MidiInCore( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::MACOSX_CORE; };
  bool hasVirtualPorts( ) const { return true; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );

public:
  void * apiData_;
  static void midiInputCallback( const MIDIPacketList * list,
                                 void * procRef,
                                 void */* srcRef*/ ) throw( );
  void initialize( const std::string& clientName );
  template<int locking>
  friend class CoreSequencer;
};

class MidiOutCore : public InternalMidiApi
{
public:
  MidiOutCore( const std::string& clientName );
  ~MidiOutCore( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::MACOSX_CORE; };
  bool hasVirtualPorts( ) const { return true; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
  void sendMessage( const unsigned char * message, size_t size );

public:
  void * apiData_;
  void initialize( const std::string& clientName );
};

#endif

#if defined( __LINUX_ALSA__ )

// class MidiInAlsa comes after AlsaMidiData

class MidiOutAlsa : public InternalMidiApi
{
public:
  MidiOutAlsa( const std::string& clientName );
  ~MidiOutAlsa( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::LINUX_ALSA; };
  bool hasVirtualPorts( ) const { return true; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
  void sendMessage( const unsigned char * message, size_t size );

public:
  void * apiData_;
  void initialize( const std::string& clientName );
};

#endif

#if defined( __WINDOWS_MM__ )

class MidiInWinMM : public InternalMidiApi
{
public:
  MidiInWinMM( const std::string& clientName );
  ~MidiInWinMM( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::WINDOWS_MM; };
  bool hasVirtualPorts( ) const { return false; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );

public:
  void * apiData_;
  void initialize( const std::string& clientName );
  friend struct WinMMCallbacks;
};

class MidiOutWinMM : public InternalMidiApi
{
public:
  MidiOutWinMM( const std::string& clientName );
  ~MidiOutWinMM( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::WINDOWS_MM; };
  bool hasVirtualPorts( ) const { return false; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
  void sendMessage( const unsigned char * message, size_t size );

public:
  void * apiData_;
  void initialize( const std::string& clientName );
};

#endif

#if defined( __RTMIDI_DUMMY__ )

#define RTMIDI_CLASSNAME "MidiInDummy"
class MidiInDummy : public InternalMidiApi
{
public:
  MidiInDummy( const std::string& /*clientName*/ )
    : InternalMidiApi( ) {
    error( RTMIDI_ERROR( rtmidi_gettext( "No valid MIDI interfaces. I'm using a dummy input interface that never receives anything." ),
                         Error::WARNING ) );
  }
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::DUMMY; }
  bool hasVirtualPorts( ) const { return false; }
  void openPort( unsigned int /*portNumber*/, const std::string& /*portName*/ ) {}
  void openVirtualPort( const std::string& /*portName*/ ) {}
  void openPort( const PortDescriptor& /* port */,
                 const std::string& /* portName */ ) {}
  Pointer<PortDescriptor> getDescriptor( bool /* local=false */ ) {
    return 0;
  }
  PortList getPortList( int /* capabilities */ ) {
    return PortList( );
  }
  void closePort( void ) {}
  void setClientName( const std::string& /*clientName*/ ) {};
  void setPortName( const std::string& /*portName*/ ) {};
  unsigned int getPortCount( void ) { return 0; }
  std::string getPortName( unsigned int /*portNumber*/ ) { return ""; }

public:
  void initialize( const std::string& /*clientName*/ ) {}
};

class MidiOutDummy : public InternalMidiApi
{
public:
  MidiOutDummy( const std::string& /*clientName*/ ) {
    error( RTMIDI_ERROR( rtmidi_gettext( "No valid MIDI interfaces. I'm using a dummy output interface that does nothing." ),
                         Error::WARNING ) );
  }
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::DUMMY; }
  bool hasVirtualPorts( ) const { return false; }
  void openPort( unsigned int /*portNumber*/, const std::string& /*portName*/ ) {}
  void openVirtualPort( const std::string& /*portName*/ ) {}
  void openPort( const PortDescriptor& /* port */,
                 const std::string& /* portName */ ) {}
  Pointer<PortDescriptor> getDescriptor( bool /* local=false */ ) {
    return 0;
  }
  PortList getPortList( int /* capabilities */ ) {
    return PortList( );
  }
  void closePort( void ) {}
  void setClientName( const std::string& /*clientName*/ ) {};
  void setPortName( const std::string& /*portName*/ ) {};
  unsigned int getPortCount( void ) { return 0; }
  std::string getPortName( unsigned int /*portNumber*/ ) { return ""; }
  void sendMessage( const unsigned char * /*message*/, size_t /*size*/ ) {}

public:
  void initialize( const std::string& /*clientName*/ ) {}
};
#undef RTMIDI_CLASSNAME

#endif


//*********************************************************************//
// RtMidi Definitions
//*********************************************************************//


// *************************************************** //
//
// OS/API-specific methods.
//
// *************************************************** //

#if defined( __MACOSX_COREMIDI__ )

// The CoreMIDI API is based on the use of a callback function for
// MIDI input. We convert the system specific time stamps to delta
// time values.

RTMIDI_NAMESPACE_END
// OS-X CoreMIDI header files.
#include <CoreMIDI/CoreMIDI.h>
#include <CoreAudio/HostTime.h>
#include <CoreServices/CoreServices.h>
RTMIDI_NAMESPACE_START

/*! An abstraction layer for the CORE sequencer layer. It provides
  the following functionality:
  - dynamic allocation of the sequencer
  - optionallay avoid concurrent access to the CORE sequencer,
  which is not thread proof. This feature is controlled by
  the parameter \ref locking.
*/

/* A wrapper for temporary CFString objects */
class CFStringWrapper {
public:
  CFStringWrapper( const std::string& s )
    : cfname( CFStringCreateWithCStringNoCopy( NULL,
                                               s.c_str( ),
                                               kCFStringEncodingUTF8,
                                               kCFAllocatorNull ) )
  {}
  CFStringWrapper( const char * s )
    : cfname( CFStringCreateWithCStringNoCopy( NULL,
                                               s,
                                               kCFStringEncodingUTF8,
                                               kCFAllocatorNull ) )
  {}
  ~CFStringWrapper( ) {
                       CFRelease( cfname );
  }

  operator CFStringRef&( ) { return cfname; }
public:
  CFStringRef cfname;
};

// This function was submitted by Douglas Casey Tucker and apparently
// derived largely from PortMidi.
// or copied from the Apple developer Q&A website
// https://developer.apple.com/library/mac/qa/qa1374/_index.html

CFStringRef EndpointName( MIDIEndpointRef endpoint, bool isExternal )
{
  CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
  CFStringRef str;

  // Begin with the endpoint's name.
  str = NULL;
  MIDIObjectGetStringProperty( endpoint, kMIDIPropertyName, &str );
  if ( str != NULL ) {
    CFStringAppend( result, str );
    CFRelease( str );
  }

  MIDIEntityRef entity = 0;
  MIDIEndpointGetEntity( endpoint, &entity );
  if ( entity == 0 )
    // probably virtual
    return result;

  if ( CFStringGetLength( result ) == 0 ) {
    // endpoint name has zero length -- try the entity
    str = NULL;
    MIDIObjectGetStringProperty( entity, kMIDIPropertyName, &str );
    if ( str != NULL ) {
      CFStringAppend( result, str );
      CFRelease( str );
    }
  }
  // now consider the device's name
  MIDIDeviceRef device = 0;
  MIDIEntityGetDevice( entity, &device );
  if ( device == 0 )
    return result;

  str = NULL;
  MIDIObjectGetStringProperty( device, kMIDIPropertyName, &str );
  if ( CFStringGetLength( result ) == 0 ) {
    CFRelease( result );
    return str;
  }
  if ( str != NULL ) {
    // if an external device has only one entity, throw away
    // the endpoint name and just use the device name
    if ( isExternal && MIDIDeviceGetNumberOfEntities( device ) < 2 ) {
      CFRelease( result );
      return str;
    } else {
      if ( CFStringGetLength( str ) == 0 ) {
        CFRelease( str );
        return result;
      }
      // does the entity name already start with the device name?
      // ( some drivers do this though they shouldn't )
      // if so, do not prepend
      if ( CFStringCompareWithOptions( result, /* endpoint name */
                                       str /* device name */,
                                       CFRangeMake( 0, CFStringGetLength( str ) ), 0 ) != kCFCompareEqualTo ) {
        // prepend the device name to the entity name
        if ( CFStringGetLength( result ) > 0 )
          CFStringInsert( result, 0, CFSTR( " " ) );
        CFStringInsert( result, 0, str );
      }
      CFRelease( str );
    }
  }
  return result;
}

// This function was submitted by Douglas Casey Tucker and apparently
// derived largely from PortMidi.
// Nearly the same text can be found in the Apple Q&A qa1374:
// https://developer.apple.com/library/mac/qa/qa1374/_index.html
static CFStringRef ConnectedEndpointName( MIDIEndpointRef endpoint )
{
  CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
  CFStringRef str;
  OSStatus err;
  int i;

  // Does the endpoint have connections?
  CFDataRef connections = NULL;
  int nConnected = 0;
  bool anyStrings = false;
  err = MIDIObjectGetDataProperty( endpoint, kMIDIPropertyConnectionUniqueID, &connections );
  if ( connections != NULL ) {
    // It has connections, follow them
    // Concatenate the names of all connected devices
    nConnected = CFDataGetLength( connections ) / sizeof( MIDIUniqueID );
    if ( nConnected ) {
      const SInt32 * pid = reinterpret_cast<const SInt32*>( CFDataGetBytePtr( connections ) );
      for ( i=0; i<nConnected; ++i, ++pid ) {
        MIDIUniqueID id = EndianS32_BtoN( *pid );
        MIDIObjectRef connObject;
        MIDIObjectType connObjectType;
        err = MIDIObjectFindByUniqueID( id, &connObject, &connObjectType );
        if ( err == noErr ) {
          if ( connObjectType == kMIDIObjectType_ExternalSource ||
               connObjectType == kMIDIObjectType_ExternalDestination ) {
            // Connected to an external device's endpoint ( 10.3 and later ).
            str = EndpointName( ( MIDIEndpointRef )( connObject ), true );
          } else {
            // Connected to an external device ( 10.2 ) ( or something else, catch-
            str = NULL;
            MIDIObjectGetStringProperty( connObject, kMIDIPropertyName, &str );
          }
          if ( str != NULL ) {
            if ( anyStrings )
              CFStringAppend( result, CFSTR( ", " ) );
            else anyStrings = true;
            CFStringAppend( result, str );
            CFRelease( str );
          }
        }
      }
    }
    CFRelease( connections );
  }
  if ( anyStrings )
    return result;

  CFRelease( result );

  // Here, either the endpoint had no connections, or we failed to obtain names
  return EndpointName( endpoint, false );
}


#define RTMIDI_CLASSNAME "CoreSequencer"
template <int locking=1>
class CoreSequencer {
public:
  CoreSequencer( )
    : seq( 0 )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
    }
  }

  CoreSequencer( const std::string& n )
    : seq( 0 ), name( n )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
    }
    init( );
  }

  ~CoreSequencer( )
  {
    if ( seq ) {
      scoped_lock<locking> lock( mutex );
      MIDIClientDispose( seq );
      seq = 0;
    }
    if ( locking ) {
      pthread_mutex_destroy( &mutex );
    }
  }

  bool setName( const std::string& n ) {
    /* we don't want to rename the client after opening it. */
    if ( seq ) return false;
    name = n;
    return true;
  }

  static std::string str( CFStringRef s ) {
    const char * cstr =
      CFStringGetCStringPtr( s, kCFStringEncodingUTF8 );
    if ( cstr ) return cstr;

    CFIndex len = CFStringGetLength( s );
    std::string retval;
    retval.resize( CFStringGetMaximumSizeForEncoding( len,
                                                      kCFStringEncodingUTF8 )+1 );
    CFStringGetBytes( s,
                      CFRangeMake( 0, len ),
                      kCFStringEncodingUTF8,
                      0,
                      false,
                      reinterpret_cast<uint8*>( &retval[0] ),
                      retval.size( )-1,
                      &len );
    retval.resize( len );
    return trim( retval );
  }


#if 0
  // Obtain the name of an endpoint, following connections.

  // The result should be released by the caller.

  static CFStringRef CreateConnectedEndpointName( MIDIEndpointRef endpoint )
  {
    CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
    CFStringRef str;
    OSStatus err;


    // Does the endpoint have connections?
    CFDataRef connections = NULL;
    int nConnected = 0;
    bool anyStrings = false;
    err = MIDIObjectGetDataProperty( endpoint, kMIDIPropertyConnectionUniqueID, &connections );
    if ( connections != NULL ) {
      // It has connections, follow them
      // Concatenate the names of all connected devices
      nConnected = CFDataGetLength( connections ) / sizeof( MIDIUniqueID );

      if ( nConnected ) {
        const SInt32 * pid = reinterpret_cast<const SInt32 *>( CFDataGetBytePtr( connections ) );
        for ( int i = 0; i < nConnected; ++i, ++pid ) {
          MIDIUniqueID id = EndianS32_BtoN( *pid );
          MIDIObjectRef connObject;
          MIDIObjectType connObjectType;
          err = MIDIObjectFindByUniqueID( id, &connObject, &connObjectType );
          if ( err == noErr ) {
            if ( connObjectType == kMIDIObjectType_ExternalSource ||
                 connObjectType == kMIDIObjectType_ExternalDestination ) {
              // Connected to an external device's endpoint ( 10.3 and later ).
              str = EndpointName( static_cast<MIDIEndpointRef>( connObject ), true );
            } else {
              // Connected to an external device ( 10.2 ) ( or something else, catch-all )
              str = NULL;
              MIDIObjectGetStringProperty( connObject, kMIDIPropertyName, &str );
            }

            if ( str != NULL ) {
              if ( anyStrings )
                CFStringAppend( result, CFSTR( ", " ) );
              else anyStrings = true;
              CFStringAppend( result, str );
              CFRelease( str );
            }
          }
        }
      }
      CFRelease( connections );
    }


    if ( anyStrings )
      return result;
    else
      CFRelease( result );

    // Here, either the endpoint had no connections, or we failed to obtain names for any of them.
    return CreateEndpointName( endpoint, false );
  }



  //////////////////////////////////////
  // Obtain the name of an endpoint without regard for whether it has connections.
  // The result should be released by the caller.

  static CFStringRef CreateEndpointName( MIDIEndpointRef endpoint, bool isExternal )
  {
    CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
    CFStringRef str;

    // begin with the endpoint's name
    str = NULL;
    MIDIObjectGetStringProperty( endpoint, kMIDIPropertyName, &str );
    if ( str != NULL ) {
      CFStringAppend( result, str );
      CFRelease( str );
    }

    MIDIEntityRef entity = NULL;
    MIDIEndpointGetEntity( endpoint, &entity );
    if ( entity == NULL )
      // probably virtual
      return result;

    if ( CFStringGetLength( result ) == 0 ) {
      // endpoint name has zero length -- try the entity
      str = NULL;
      MIDIObjectGetStringProperty( entity, kMIDIPropertyName, &str );
      if ( str != NULL ) {
        CFStringAppend( result, str );
        CFRelease( str );
      }
    }



    // now consider the device's name
    MIDIDeviceRef device = NULL;
    MIDIEntityGetDevice( entity, &device );
    if ( device == NULL ) return result;

    str = NULL;
    MIDIObjectGetStringProperty( device, kMIDIPropertyName, &str );
    if ( str != NULL ) {
      // if an external device has only one entity, throw away
      // the endpoint name and just use the device name
      if ( isExternal && MIDIDeviceGetNumberOfEntities( device ) < 2 ) {
        CFRelease( result );
        return str;
      } else {
        // does the entity name already start with the device name?
        // ( some drivers do this though they shouldn't )
        // if so, do not prepend

        if ( CFStringCompareWithOptions( str /* device name */,
                                         result /* endpoint name */,
                                         CFRangeMake( 0,
                                                      CFStringGetLength( str ) ),
                                         0 )
             != kCFCompareEqualTo ) {
          // prepend the device name to the entity name
          if ( CFStringGetLength( result ) > 0 )
            CFStringInsert( result, 0, CFSTR( " " ) );
          CFStringInsert( result, 0, str );
        }
        CFRelease( str );
      }
    }

    return result;
  }
#endif

  static std::string getConnectionsString( MIDIEndpointRef port )
  {
    /* This function is derived from
       CreateConnectedEndpointName at Apple Q&A */
    std::ostringstream result;
    CFDataRef connections = NULL;
    OSStatus err = MIDIObjectGetDataProperty( port,
                                              kMIDIPropertyConnectionUniqueID,
                                              &connections );
    if ( err != noErr )
      return result.str( );

    if ( !connections )
      return result.str( );
    CFIndex size = CFDataGetLength( connections ) / sizeof( MIDIUniqueID );
    if ( !size ) {
      CFRelease( connections );
      return result.str( );
    }

    CFStringRef strRef;
    const SInt32 * pid
      = reinterpret_cast<const SInt32 *>( CFDataGetBytePtr( connections ) );
    bool anyStrings = false;
    for ( int i = 0; i < size; ++i, ++pid ) {
      MIDIUniqueID id = EndianS32_BtoN( *pid );
      MIDIObjectRef connObject;
      MIDIObjectType connObjectType;
      err = MIDIObjectFindByUniqueID( id, &connObject, &connObjectType );
      if ( err != noErr )
        continue;

      if ( connObjectType == kMIDIObjectType_ExternalSource ||
           connObjectType == kMIDIObjectType_ExternalDestination ) {
        // Connected to an external
        // device's endpoint
        // ( 10.3 and later ).
        strRef = EndpointName( static_cast<MIDIEndpointRef>( connObject ),
                               true );
      } else {
        // Connected to an external device
        // ( 10.2 ) ( or something else, catch-all )
        strRef = NULL;
        MIDIObjectGetStringProperty( connObject,
                                     kMIDIPropertyName, &strRef );
      }

      if ( strRef != NULL ) {
        if ( anyStrings )
          result << ", ";
        else anyStrings = true;
        result << str( strRef );
        CFRelease( strRef );
      }
    }
    CFRelease( connections );
    return result.str( );
  }

  static std::string getPortName( MIDIEndpointRef port, int flags ) {
    //   std::string clientname;
    std::string devicename;
    std::string portname;
    std::string entityname;
    //   std::string externaldevicename;
    std::string connections;
    std::string recommendedname;
    //   bool isVirtual;
    bool hasManyEntities = false;
    bool hasManyEndpoints = false;
    CFStringRef nameRef;
    MIDIObjectGetStringProperty( port,
                                 kMIDIPropertyDisplayName,
                                 &nameRef );
    recommendedname = str( nameRef );
    connections = getConnectionsString( port );

    MIDIObjectGetStringProperty( port,
                                 kMIDIPropertyName,
                                 &nameRef );
    portname = str( nameRef );
    CFRelease( nameRef );

    MIDIEntityRef entity = 0;
    MIDIEndpointGetEntity( port, &entity );
    // entity == NULL: probably virtual
    if ( entity != 0 ) {
      nameRef = NULL;
      MIDIObjectGetStringProperty( entity, kMIDIPropertyName, &nameRef );
      if ( nameRef != NULL ) {
        entityname = str( nameRef );
        CFRelease( nameRef );
      }
      hasManyEndpoints =
        MIDIEntityGetNumberOfSources( entity ) >= 2 ||
        MIDIEntityGetNumberOfDestinations( entity )
        >= 2;

      // now consider the device's name
      MIDIDeviceRef device = 0;
      MIDIEntityGetDevice( entity, &device );
      if ( device != 0 ) {
        hasManyEntities = MIDIDeviceGetNumberOfEntities( device ) >= 2;
        MIDIObjectGetStringProperty( device,
                                     kMIDIPropertyName,
                                     &nameRef );
        devicename = str( nameRef );
        CFRelease( nameRef );
      }
      // does the entity name already start with the device name?
      // ( some drivers do this though they shouldn't )
      if ( entityname.substr( 0, devicename.length( ) )
           == devicename ) {
        int start = devicename.length( );
        while ( isspace( entityname[start] ) )
          start++;
        entityname = entityname.substr( start );
      }
    }

    int naming = flags & PortDescriptor::NAMING_MASK;

    std::ostringstream os;
    bool needcolon;
    switch ( naming ) {
    case PortDescriptor::SESSION_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "CORE:";
      os << port;
      break;
    case PortDescriptor::STORAGE_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "CORE:";
      // os << clientname;
      os << devicename;
      os << ":" << portname;
      os << ":" << entityname;
      //    os << ":" << externaldevicename;
      os << ":" << connections;
      //    os << ":" << recommendedname;
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME )
        os << ";" << port;
      break;
    case PortDescriptor::LONG_NAME:
      needcolon = !devicename.empty( );
      os << devicename;
      if ( hasManyEndpoints ||
           hasManyEntities ||
           devicename.empty( ) ) {
        if ( !entityname.empty( ) ) {
          if ( needcolon )
            os << ": ";
          os << entityname;
          needcolon = true;
        }
        if ( ( hasManyEndpoints
               || entityname.empty( ) )
             && !portname.empty( ) ) {
          if ( needcolon )
            os << ": ";
          os << portname;
        }
      }
      if ( !connections.empty( ) ) {
        os << " ⇒ ";
        os << connections;
      }
      if ( flags &
           ( PortDescriptor::INCLUDE_API
             | PortDescriptor::UNIQUE_PORT_NAME ) ) {
        os << " ( ";
        if ( flags &
             PortDescriptor::INCLUDE_API ) {
          os << "CORE";
          if ( flags & PortDescriptor::UNIQUE_PORT_NAME )
            os << ":";
        }
        if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
          os << port;
        }
        os << " )";
      }
      break;
    case PortDescriptor::SHORT_NAME:
    default:
      if ( !recommendedname.empty( ) ) {
        os << recommendedname;
      } else
        if ( !connections.empty( ) ) {
          os << connections;
        } else {
          os << devicename;
          if ( hasManyEntities ||
               hasManyEndpoints ||
               devicename.empty( ) ) {
            if ( !devicename.empty( ) )
              os << " ";
            if ( !portname.empty( ) ) {
              os << portname;
            } else if ( !entityname.empty( ) ) {
              os << entityname;
            } else
              os << "???";
          }
        }
      if ( flags &
           ( PortDescriptor::INCLUDE_API
             | PortDescriptor::UNIQUE_PORT_NAME ) ) {
        os << " ( ";
        if ( flags &
             PortDescriptor::INCLUDE_API ) {
          os << "CORE";
          if ( flags & PortDescriptor::UNIQUE_PORT_NAME )
            os << ":";
        }
        if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
          os << port;
        }
        os << " )";
      }
      break;
    }
    return os.str( );
  }

  int getPortCapabilities( MIDIEndpointRef port ) {
    int retval = 0;
    MIDIEntityRef entity = 0;
    OSStatus stat =
      MIDIEndpointGetEntity( port, &entity );
    if ( stat == kMIDIObjectNotFound ) {
      // plan B for virtual ports
      MIDIUniqueID uid;
      stat = MIDIObjectGetIntegerProperty ( port,
                                            kMIDIPropertyUniqueID,
                                            &uid );
      if ( stat != noErr ) {
        throw RTMIDI_ERROR( gettext_noopt( "Could not get the unique identifier of a midi endpoint." ),
                            Error::WARNING );
        return 0;
      }
      MIDIObjectRef obj;
      MIDIObjectType type;
      stat = MIDIObjectFindByUniqueID ( uid,
                                        &obj,
                                        &type );
      if ( stat != noErr || obj != port ) {
        throw RTMIDI_ERROR( gettext_noopt( "Could not get the endpoint back from the unique identifier of a midi endpoint." ),
                            Error::WARNING );
        return 0;
      }
      if ( type == kMIDIObjectType_Source
           || type == kMIDIObjectType_ExternalSource )
        return PortDescriptor::INPUT;
      else if ( type == kMIDIObjectType_Destination
                || type == kMIDIObjectType_ExternalDestination )
        return PortDescriptor::OUTPUT;
      else {
        return 0;
      }

    } else if ( stat != noErr ) {
      throw RTMIDI_ERROR( gettext_noopt( "Could not get the entity of a midi endpoint." ),
                          Error::WARNING );
      return 0;
    }
    /* Theoretically Mac OS X could silently use
       the same endpoint reference for input and
       output. We might benefit from this
       behaviour.
       \todo: Find a way to query the object
       whether it can act as source or destination.
    */
    ItemCount count =
      MIDIEntityGetNumberOfDestinations( entity );
    for ( ItemCount i = 0; i < count ; i++ ) {
      MIDIEndpointRef dest=
        MIDIEntityGetDestination( entity, i );
      if ( dest == port ) {
        retval |=
          PortDescriptor::OUTPUT;
        break;
      }
    }
    count =
      MIDIEntityGetNumberOfSources( entity );
    for ( ItemCount i = 0; i < count ; i++ ) {
      MIDIEndpointRef src=
        MIDIEntityGetSource( entity, i );
      if ( src == port ) {
        retval |=
          PortDescriptor::INPUT;
      }
    }
    return retval;
  }


  MIDIPortRef createPort ( const std::string& portName,
                           int flags,
                           MidiInCore * data = NULL )
  {
    init( );
    scoped_lock<locking> lock ( mutex );
    MIDIPortRef port = 0;
    OSStatus result;
    switch ( flags ) {
    case PortDescriptor::INPUT: {
      result = MIDIInputPortCreate( seq,
                                    CFStringWrapper( portName ),
                                    MidiInCore::midiInputCallback,
                                    (void *)data,
                                    &port );
    }
      break;
    case PortDescriptor::OUTPUT: {
      result
        = MIDIOutputPortCreate( seq,
                                CFStringWrapper( portName ),
                                &port );
    }
      break;
    default:
      throw RTMIDI_ERROR( gettext_noopt( "Error creating OS X MIDI port because of invalid port flags." ),
                          Error::INVALID_PARAMETER );
    }
    if ( result != noErr ) {
      throw RTMIDI_ERROR( gettext_noopt( "Error creating OS-X MIDI port." ),
                          Error::DRIVER_ERROR );
    }
    return port;
  }

  MIDIEndpointRef createVirtualPort ( const std::string& portName,
                                      int flags,
                                      MidiInCore * data = NULL )
  {
    init( );
    scoped_lock<locking> lock ( mutex );
    MIDIEndpointRef port = 0;
    OSStatus result;
    switch ( flags ) {
    case PortDescriptor::INPUT: {
      result
        = MIDIDestinationCreate( seq,
                                 CFStringWrapper( portName ),
                                 MidiInCore::midiInputCallback,
                                 (void *)data,
                                 &port );
    }
      break;
    case PortDescriptor::OUTPUT: {
      result
        = MIDISourceCreate( seq,
                            CFStringWrapper( portName ),
                            &port );
    }
      break;
    default:
      throw RTMIDI_ERROR( gettext_noopt( "Error creating OS X MIDI port because of invalid port flags." ),
                          Error::INVALID_PARAMETER );
    }
    if ( result != noErr ) {
      throw RTMIDI_ERROR( gettext_noopt( "Error creating OS-X MIDI port." ),
                          Error::DRIVER_ERROR );
    }
    return port;
  }


  /*! Use CoreSequencer like a C pointer.
    \note This function breaks the design to control thread safety
    by the selection of the \ref locking parameter to the class.
    It should be removed as soon as possible in order ensure the
    thread policy that has been intended by creating this class.
  */
  operator MIDIClientRef ( )
  {
    return seq;
  }
public:
  pthread_mutex_t mutex;
  MIDIClientRef seq;
  std::string name;


  void init( )
  {
    init ( seq );
  }

  void init( MIDIClientRef& client )
  {
    if ( client ) return;
    scoped_lock<locking> lock( mutex );

    OSStatus result = MIDIClientCreate( CFStringWrapper( name ), NULL, NULL, &client );
    if ( result != noErr ) {
      throw RTMIDI_ERROR1( gettext_noopt( "Error creating OS-X MIDI client object ( Error no: %d )." ),
                           Error::DRIVER_ERROR,
                           result );
      return;
    }
  }
};
#undef RTMIDI_CLASSNAME

typedef CoreSequencer<1> LockingCoreSequencer;
typedef CoreSequencer<0> NonLockingCoreSequencer;

#define RTMIDI_CLASSNAME "CorePortDescriptor"
struct CorePortDescriptor: public PortDescriptor {
  CorePortDescriptor( const std::string& name )
    : api( 0 ),
      clientName( name ),
      endpoint( 0 )
  {
  }
  CorePortDescriptor( MIDIEndpointRef p,
                      const std::string& name )
    : api( 0 ),
      clientName( name ),
      endpoint( p )
  {
    seq.setName( name );
  }
  CorePortDescriptor( CorePortDescriptor& other )
    : PortDescriptor( other ),
      api( other.api ),
      clientName( other.clientName ),
      endpoint( other.endpoint )
  {
    seq.setName( clientName );
  }
  ~CorePortDescriptor( ) {}

  InternalMidiApi * getInputApi( ) const {
    if ( getCapabilities( ) & INPUT )
      return new MidiInCore( clientName );
    else
      return 0;
  }

  InternalMidiApi * getOutputApi( ) const {
    if ( getCapabilities( ) & OUTPUT )
      return new MidiOutCore( clientName );
    else
      return 0;
  }

  void setEndpoint( MIDIEndpointRef e )
  {
    endpoint = e;
  }
  MIDIEndpointRef getEndpoint( ) const
  {
    return endpoint;
  }

  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq.getPortName( endpoint, flags );
  }

  const std::string& getClientName( ) {
    return clientName;
  }
  int getCapabilities( ) const {
    if ( !endpoint ) return 0;
    return seq.getPortCapabilities( endpoint );
  }

  virtual bool operator == ( const PortDescriptor& o ) {
    const CorePortDescriptor * desc = dynamic_cast<const CorePortDescriptor*>( &o );
    if ( !desc ) return false;
    return endpoint == desc->endpoint;
  }
  static PortList getPortList( int capabilities, const std::string& clientName );
public:
  MidiApi * api;
  static LockingCoreSequencer seq;

  std::string clientName;
  MIDIEndpointRef endpoint;
};

LockingCoreSequencer CorePortDescriptor :: seq;



PortList CorePortDescriptor :: getPortList( int capabilities, const std::string& clientName )
{
  PortList list;

  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  int caps = capabilities & PortDescriptor::INOUTPUT;
  // bool unlimited = capabilities & PortDescriptor::UNLIMITED;
  bool forceInput = PortDescriptor::INPUT & caps;
  bool forceOutput = PortDescriptor::OUTPUT & caps;
  bool allowOutput = forceOutput || !forceInput;
  bool allowInput = forceInput || !forceOutput;
  if ( allowOutput ) {
    ItemCount count =
      MIDIGetNumberOfDestinations( );
    for ( ItemCount i = 0 ; i < count; i++ ) {
      MIDIEndpointRef destination =
        MIDIGetDestination( i );
      try {
        if ( ( seq.getPortCapabilities( destination )
               & caps ) == caps )
          list.push_back( Pointer<PortDescriptor>(
                                                  new CorePortDescriptor( destination, clientName ) ) );
      } catch ( Error& e ) {
        if ( e.getType( ) == Error::WARNING ||
             e.getType( ) == Error::DEBUG_WARNING )
          e.printMessage( );
        else throw;
      }
    }
    // Combined sources and destinations
    // should be both occur as destinations and as
    // sources. So we have finished the search, here.
  } else if ( allowInput ) {
    ItemCount count =
      MIDIGetNumberOfSources( );
    for ( ItemCount i = 0 ; i < count; i++ ) {
      MIDIEndpointRef src =
        MIDIGetSource( i );
      try {
        if ( ( seq.getPortCapabilities( src )
               & caps ) == caps )
          list.push_back( Pointer<PortDescriptor>(
                                                  new CorePortDescriptor( src, clientName ) ) );
      } catch ( Error& e ) {
        if ( e.getType( ) == Error::WARNING ||
             e.getType( ) == Error::DEBUG_WARNING )
          e.printMessage( );
        else throw;
      }
    }
  }
  return list;
}
#undef RTMIDI_CLASSNAME


#define RTMIDI_CLASSNAME "CoreMidiData"
// A structure to hold variables related to the CoreMIDI API
// implementation.
struct CoreMidiData: public CorePortDescriptor {
  CoreMidiData( const std::string& clientname )
    : CorePortDescriptor( clientname ),
      client( clientname ),
      localEndpoint( 0 ),
      localPort( 0 ) {}
  ~CoreMidiData( ) {
    if ( localEndpoint )
      MIDIEndpointDispose( localEndpoint );
    localEndpoint = 0;
  }

  void openPort( const std::string& name,
                 int flags,
                 MidiInCore * data = NULL ) {
    localPort = client.createPort( name, flags, data );
  }

  void setRemote( const CorePortDescriptor& remote )
  {
    setEndpoint( remote.getEndpoint( ) );
  }

  NonLockingCoreSequencer client;
  MIDIEndpointRef localEndpoint;
  MIDIPortRef localPort;
  unsigned long long lastTime;
  MIDISysexSendRequest sysexreq;
};
#undef RTMIDI_CLASSNAME


//*********************************************************************//
// API: OS-X
// Class Definitions: MidiInCore
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiInCore"
void MidiInCore :: midiInputCallback( const MIDIPacketList * list,
                                      void * procRef,
                                      void */*srcRef*/ ) throw( )
{
  MidiInCore * data = static_cast<MidiInCore *> ( procRef );
  CoreMidiData * apiData = static_cast<CoreMidiData *> ( data->apiData_ );

  unsigned char status;
  unsigned short nBytes, iByte, size;
  unsigned long long time;

  bool& continueSysex = data->continueSysex;
  MidiMessage& message = data->message;

  const MIDIPacket * packet = &list->packet[0];
  for ( unsigned int i=0; i<list->numPackets; ++i ) {

    // My interpretation of the CoreMIDI documentation: all message
    // types, except sysex, are complete within a packet and there may
    // be several of them in a single packet. Sysex messages can be
    // broken across multiple packets and PacketLists but are bundled
    // alone within each packet ( these packets do not contain other
    // message types ). If sysex messages are split across multiple
    // MIDIPacketLists, they must be handled by multiple calls to this
    // function.

    nBytes = packet->length;
    if ( nBytes == 0 ) {
      packet = MIDIPacketNext( packet );
      continue;
    }

    // Calculate time stamp.

    if ( data->firstMessage ) {
      message.timeStamp = 0.0;
      data->firstMessage = false;
    }
    else {
      time = packet->timeStamp;
      if ( time == 0 ) { // this happens when receiving asynchronous sysex messages
        time = AudioGetCurrentHostTime( );
      }
      time -= apiData->lastTime;
      time = AudioConvertHostTimeToNanos( time );
      if ( !continueSysex )
        message.timeStamp = time * 0.000000001;
    }
    // Track whether any non-filtered messages were found in this
    // packet for timestamp calculation
    bool foundNonFiltered = false;
    //std::cout << "TimeStamp = " << packet->timeStamp << std::endl;

    iByte = 0;
    if ( continueSysex ) {
      // We have a continuing, segmented sysex message.
      if ( !( data->ignoreFlags & IGNORE_SYSEX ) ) {
        // If we're not ignoring sysex messages, copy the entire packet.
        for ( unsigned int j=0; j<nBytes; ++j )
          message.bytes.push_back( packet->data[j] );
      }
      continueSysex = packet->data[nBytes-1] != 0xF7;

      if ( !( data->ignoreFlags & IGNORE_SYSEX ) ) {
        if ( !continueSysex ) {
          // If not a continuing sysex message, invoke the user callback function or queue the message.
          if ( data->userCallback ) {
            data->userCallback->rtmidi_midi_in( message.timeStamp,
                                                message.bytes );
          }
          else {
            // As long as we haven't reached our queue size limit, push the message.
            if ( !data->queue.push( message ) ) {
              try {
                data->error( RTMIDI_ERROR( rtmidi_gettext( "Error: Message queue limit reached." ),
                                           Error::WARNING ) );
              } catch ( Error& e ) {
                // don't bother ALSA with an unhandled exception
              }
            }
          }
          message.bytes.clear( );
        }
      }
    }
    else {
      while ( iByte < nBytes ) {
        size = 0;
        // We are expecting that the next byte in the packet is a status byte.
        status = packet->data[iByte];
        if ( !( status & 0x80 ) ) break;
        // Determine the number of bytes in the MIDI message.
        if ( status < 0xC0 ) size = 3;
        else if ( status < 0xE0 ) size = 2;
        else if ( status < 0xF0 ) size = 3;
        else if ( status == 0xF0 ) {
          // A MIDI sysex
          if ( data->ignoreFlags & IGNORE_SYSEX ) {
            size = 0;
            iByte = nBytes;
          }
          else size = nBytes - iByte;
          continueSysex = packet->data[nBytes-1] != 0xF7;
        }
        else if ( status == 0xF1 ) {
          // A MIDI time code message
          if ( data->ignoreFlags & IGNORE_TIME ) {
            size = 0;
            iByte += 2;
          }
          else size = 2;
        }
        else if ( status == 0xF2 ) size = 3;
        else if ( status == 0xF3 ) size = 2;
        else if ( status == 0xF8 && ( data->ignoreFlags & IGNORE_TIME ) ) {
          // A MIDI timing tick message and we're ignoring it.
          size = 0;
          iByte += 1;
        }
        else if ( status == 0xFE && ( data->ignoreFlags & IGNORE_SENSING ) ) {
          // A MIDI active sensing message and we're ignoring it.
          size = 0;
          iByte += 1;
        }
        else size = 1;

        // Copy the MIDI data to our vector.
        if ( size ) {
          foundNonFiltered = true;
          message.bytes.assign( &packet->data[iByte], &packet->data[iByte+size] );
          if ( !continueSysex ) {
            // If not a continuing sysex message, invoke the user callback function or queue the message.
            if ( data->userCallback ) {
              data->userCallback->rtmidi_midi_in( message.timeStamp,
                                                  message.bytes );
            }
            else {
              // As long as we haven't reached our queue size limit, push the message.
              if ( !data->queue.push( message ) ) {
                try {
                  data->error( RTMIDI_ERROR( rtmidi_gettext( "Error: Message queue limit reached." ),
                                             Error::WARNING ) );
                } catch ( Error& e ) {
                  // don't bother WinMM with an unhandled exception
                }
              }
            }
            message.bytes.clear( );
          }
          iByte += size;
        }
      }
    }

    // Save the time of the last non-filtered message
    if ( foundNonFiltered ) {
      apiData->lastTime = packet->timeStamp;
      if ( apiData->lastTime == 0 ) { // this happens when receiving asynchronous sysex messages
        apiData->lastTime = AudioGetCurrentHostTime( );
      }
    }

    packet = MIDIPacketNext( packet );
  }
}

MidiInCore :: MidiInCore( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiInCore::initialize( clientName );
}

MidiInCore :: ~MidiInCore( void )
{
  // Cleanup.
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  try {
    // Close a connection if it exists.
    MidiInCore::closePort( );

  } catch ( Error& e ) {
    delete data;
    throw;
  }
  delete data;

}

void MidiInCore :: initialize( const std::string& clientName )
{
  // Save our api-specific connection information.
  CoreMidiData * data = new CoreMidiData( clientName );
  apiData_ = (void *) data;
}

void MidiInCore :: openPort( unsigned int portNumber,
                             const std::string& portName )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  unsigned int nSrc = MIDIGetNumberOfSources( );
  if ( nSrc < 1 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI input sources found." ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  if ( portNumber >= nSrc ) {
    std::ostringstream ost;
    ost << "";
    errorString_ = ost.str( );
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return;
  }

  MIDIPortRef port;
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  OSStatus result = MIDIInputPortCreate( data->client,
                                         CFStringWrapper( portName ),
                                         midiInputCallback, (void *)this, &port );
  if ( result != noErr ) {
    MIDIClientDispose( data->client );
    error( RTMIDI_ERROR( gettext_noopt( "Error creating OS-X MIDI input port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Get the desired input source identifier.
  MIDIEndpointRef endpoint = MIDIGetSource( portNumber );
  if ( endpoint == 0 ) {
    MIDIPortDispose( port );
    port = 0;
    MIDIClientDispose( data->client );
    error( RTMIDI_ERROR( gettext_noopt( "Error getting MIDI input source reference." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Make the connection.
  result = MIDIPortConnectSource( port, endpoint, NULL );
  if ( result != noErr ) {
    MIDIPortDispose( port );
    port = 0;
    MIDIClientDispose( data->client );
    error( RTMIDI_ERROR( gettext_noopt( "Error connecting OS-X MIDI input port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Save our api-specific port information.
  data->localPort = port;
  data->setEndpoint( endpoint );

  connected_ = true;
}

void MidiInCore :: openVirtualPort( const std::string& portName )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );

  // Create a virtual MIDI input destination.
  MIDIEndpointRef endpoint;
  OSStatus result = MIDIDestinationCreate( data->client,
                                           CFStringWrapper( portName ),
                                           midiInputCallback, (void *)this, &endpoint );
  if ( result != noErr ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating virtual OS-X MIDI destination." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Save our api-specific connection information.
  data->localEndpoint = endpoint;
}

void MidiInCore :: openPort( const PortDescriptor& port,
                             const std::string& portName )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  const CorePortDescriptor * remote = dynamic_cast<const CorePortDescriptor *>( &port );

  if ( !data ) {
    error( RTMIDI_ERROR( gettext_noopt( "Data has not been allocated." ),
                         Error::SYSTEM_ERROR ) );
    return;
  }
  if ( connected_ || data -> localEndpoint ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( !remote ) {
    error( RTMIDI_ERROR( gettext_noopt( "Core MIDI has been instructed to open a non-Core MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  data->openPort ( portName,
                   PortDescriptor::INPUT,
                   this );
  data->setRemote( *remote );
  OSStatus result =
    MIDIPortConnectSource( data->localPort,
                           data->getEndpoint( ),
                           NULL );
  if ( result != noErr ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating OS-X MIDI port." ),
                         Error::DRIVER_ERROR ) );
  }

  connected_ = true;
}

Pointer<PortDescriptor> MidiInCore :: getDescriptor( bool isLocal )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  if ( !data ) {
    return NULL;
  }
  if ( isLocal ) {
    if ( data && data->localEndpoint ) {
      return Pointer<PortDescriptor>( new
                                      CorePortDescriptor( data->localEndpoint, data->getClientName( ) ) );
    }
  } else {
    if ( data->getEndpoint( ) ) {
      return Pointer<PortDescriptor>( new CorePortDescriptor( *data ) );
    }
  }
  return NULL;
}

PortList MidiInCore :: getPortList( int capabilities )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  try {
    return CorePortDescriptor :: getPortList( capabilities | PortDescriptor::INPUT,
                                              data->getClientName( ) );
  } catch ( Error& e ) {
    error( e );
    return PortList( );
  }
}

void MidiInCore :: closePort( void )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );

  if ( data->localPort ) {
    MIDIPortDispose( data->localPort );
    data->localPort = 0;
  }

  if ( data->localEndpoint ) {
    MIDIEndpointDispose( data->localEndpoint );
    data->localEndpoint = 0;
  }

  connected_ = false;
}

void MidiInCore :: setClientName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting client names is not implemented for Mac OS X CoreMIDI." ),
                       Error::WARNING ) );
}

void MidiInCore :: setPortName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting port names is not implemented for Mac OS X CoreMIDI." ),
                       Error::WARNING ) );
}

unsigned int MidiInCore :: getPortCount( )
{
  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  return MIDIGetNumberOfSources( );
}

std::string MidiInCore :: getPortName( unsigned int portNumber )
{
  CFStringRef nameRef;
  MIDIEndpointRef portRef;
  char name[128];

  std::string stringName;
  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  if ( portNumber >= MIDIGetNumberOfSources( ) ) {
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::WARNING, portNumber ) );
    return stringName;
  }

  portRef = MIDIGetSource( portNumber );
  nameRef = ConnectedEndpointName( portRef );
  CFStringGetCString( nameRef, name, sizeof( name ), kCFStringEncodingUTF8 );
  CFRelease( nameRef );

  return stringName = name;
}
#undef RTMIDI_CLASSNAME


//*********************************************************************//
// API: OS-X
// Class Definitions: MidiOutCore
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiOutCore"
MidiOutCore :: MidiOutCore( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiOutCore::initialize( clientName );
}

MidiOutCore :: ~MidiOutCore( void )
{
  // Close a connection if it exists.
  MidiOutCore::closePort( );

  // Cleanup.
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  delete data;
}

void MidiOutCore :: initialize( const std::string& clientName )
{
  // Save our api-specific connection information.
  CoreMidiData * data = new CoreMidiData( clientName );
  apiData_ = (void *) data;
}

unsigned int MidiOutCore :: getPortCount( )
{
  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  return MIDIGetNumberOfDestinations( );
}

std::string MidiOutCore :: getPortName( unsigned int portNumber )
{
  CFStringRef nameRef;
  MIDIEndpointRef portRef;
  char name[128];

  std::string stringName;
  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  if ( portNumber >= MIDIGetNumberOfDestinations( ) ) {
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::WARNING, portNumber ) );
    return stringName;
  }

  portRef = MIDIGetDestination( portNumber );
  nameRef = ConnectedEndpointName( portRef );
  CFStringGetCString( nameRef, name, sizeof( name ), kCFStringEncodingUTF8 );
  CFRelease( nameRef );

  return stringName = name;
}

void MidiOutCore :: openPort( unsigned int portNumber,
                              const std::string& portName )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  CFRunLoopRunInMode( kCFRunLoopDefaultMode, 0, false );
  unsigned int nDest = MIDIGetNumberOfDestinations( );
  if ( nDest < 1 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI output destinations found." ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  if ( portNumber >= nDest ) {
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return;
  }

  MIDIPortRef port;
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  OSStatus result = MIDIOutputPortCreate( data->client,
                                          CFStringWrapper( portName ),
                                          &port );
  if ( result != noErr ) {
    MIDIClientDispose( data->client );
    error( RTMIDI_ERROR( gettext_noopt( "Error creating OS-X MIDI output port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Get the desired output port identifier.
  MIDIEndpointRef destination = MIDIGetDestination( portNumber );
  if ( destination == 0 ) {
    MIDIPortDispose( port );
    port = 0;
    MIDIClientDispose( data->client );
    error( RTMIDI_ERROR( gettext_noopt( "Error getting MIDI output destination reference." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Save our api-specific connection information.
  data->localPort = port;
  data->setEndpoint( destination );
  connected_ = true;
}

void MidiOutCore :: closePort( void )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );

  if ( data->localPort ) {
    MIDIPortDispose( data->localPort );
    data->localPort = 0;
  }

  if ( data->localEndpoint ) {
    MIDIEndpointDispose( data->localEndpoint );
    data->localEndpoint = 0;
  }

  connected_ = false;
}

void MidiOutCore :: setClientName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting client names is not implemented for Mac OS X CoreMIDI." ),
                       Error::WARNING ) );
}

void MidiOutCore :: setPortName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting port names is not implemented for Mac OS X CoreMIDI." ),
                       Error::WARNING ) );
}

void MidiOutCore :: openVirtualPort( const std::string& portName )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );

  if ( data->localEndpoint ) {
    error( RTMIDI_ERROR( gettext_noopt( "A virtual output port already exists." ),
                         Error::WARNING ) );
    return;
  }

  // Create a virtual MIDI output source.
  MIDIEndpointRef endpoint;
  OSStatus result = MIDISourceCreate( data->client,
                                      CFStringWrapper( portName ),
                                      &endpoint );
  if ( result != noErr ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating OS-X virtual MIDI source." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Save our api-specific connection information.
  data->localEndpoint = endpoint;
}

void MidiOutCore :: openPort( const PortDescriptor& port,
                              const std::string& portName )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  const CorePortDescriptor * remote = dynamic_cast<const CorePortDescriptor *>( &port );

  if ( !data ) {
    error( RTMIDI_ERROR( gettext_noopt( "Data has not been allocated." ),
                         Error::SYSTEM_ERROR ) );
    return;
  }
  if ( connected_ || data -> localEndpoint ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( !remote ) {
    error( RTMIDI_ERROR( gettext_noopt( "Core MIDI has been instructed to open a non-Core MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  try {
    data->openPort ( portName,
                     PortDescriptor::OUTPUT );
    data->setRemote( *remote );
    connected_ = true;
  } catch ( Error& e ) {
    error( e );
  }
}

Pointer<PortDescriptor> MidiOutCore :: getDescriptor( bool isLocal )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  if ( !data ) {
    return NULL;
  }
  try {
    if ( isLocal ) {
      if ( data && data->localEndpoint ) {
        return Pointer<PortDescriptor>( new CorePortDescriptor( data->localEndpoint,
                                                                data->getClientName( ) ) );
      }
    } else {
      if ( data->getEndpoint( ) ) {
        return Pointer<PortDescriptor>(new CorePortDescriptor( *data ) );
      }
    }
  } catch ( Error& e ) {
    error( e );
  }
  return NULL;
}

PortList MidiOutCore :: getPortList( int capabilities )
{
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  try {
    return CorePortDescriptor :: getPortList( capabilities | PortDescriptor::OUTPUT,
                                              data->getClientName( ) );
  } catch ( Error& e ) {
    error( e );
    return PortList( );
  }
}


// Not necessary if we don't treat sysex messages any differently than
// normal messages ... see below.
//static void sysexCompletionProc( MIDISysexSendRequest *sreq )
//{
// free( sreq );
//}

void MidiOutCore :: sendMessage( const unsigned char * message, size_t size )
{
  // We use the MIDISendSysex( ) function to asynchronously send sysex
  // messages. Otherwise, we use a single CoreMIDI MIDIPacket.
  if ( size == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No data in message argument." ),
                         Error::WARNING ) );
    return;
  }

  // unsigned int packetBytes, bytesLeft = size;
  // unsigned int messageIndex = 0;
  MIDITimeStamp timeStamp = AudioGetCurrentHostTime( );
  CoreMidiData * data = static_cast<CoreMidiData *> ( apiData_ );
  OSStatus result;

  if ( message[0] != 0xF0 && size > 3 ) {
    error( RTMIDI_ERROR( gettext_noopt( "message format problem ... not sysex but > 3 bytes?" ),
                         Error::WARNING ) );
    return;
  }

  Byte buffer[size+( sizeof( MIDIPacketList ) )];
  ByteCount listSize = sizeof( buffer );
  MIDIPacketList * packetList = (MIDIPacketList*) buffer;
  MIDIPacket * packet = MIDIPacketListInit( packetList );

  ByteCount remainingBytes = size;
  while ( remainingBytes && packet ) {
    ByteCount bytesForPacket = remainingBytes > 65535 ? 65535 : remainingBytes; // 65535 = maximum size of a MIDIPacket
    const Byte* dataStartPtr = (const Byte*) &message[size - remainingBytes];
    packet = MIDIPacketListAdd( packetList, listSize, packet, timeStamp, bytesForPacket, dataStartPtr );
    remainingBytes -= bytesForPacket;
  }

  if ( !packet ) {
    error( RTMIDI_ERROR( gettext_noopt( "Could not allocate packet list." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Send to any destinations that may have connected to us.
  if ( data->localEndpoint ) {
    result = MIDIReceived( data->localEndpoint, packetList );
    if ( result != noErr ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error sending MIDI to virtual destinations." ),
                           Error::WARNING ) );
    }
  }

  // And send to an explicit destination port if we're connected.
  if ( connected_ ) {
    result = MIDISend( data->localPort, data->getEndpoint( ), packetList );
    if ( result != noErr ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error sending MIDI message to port." ),
                           Error::WARNING ) );
    }
  }
}
#undef RTMIDI_CLASSNAME
#endif // __MACOSX_COREMIDI__


//*********************************************************************//
// API: LINUX ALSA SEQUENCER
//*********************************************************************//

// API information found at:
// - http://www.alsa-project.org/documentation.php#Library

#if defined( __LINUX_ALSA__ )

// The ALSA Sequencer API is based on the use of a callback function for
// MIDI input.
//
// Thanks to Pedro Lopez-Cabanillas for help with the ALSA sequencer
// time stamps and other assorted fixes!!!

// If you don't need timestamping for incoming MIDI events, define the
// preprocessor definition AVOID_TIMESTAMPING to save resources
// associated with the ALSA sequencer queues.

RTMIDI_NAMESPACE_END

#include <pthread.h>
#include <sys/time.h>

// ALSA header file.
#include <alsa/asoundlib.h>

RTMIDI_NAMESPACE_START
struct AlsaMidiData;

/*! An abstraction layer for the ALSA sequencer layer. It provides
  the following functionality:
  - dynamic allocation of the sequencer
  - optionallay avoid concurrent access to the ALSA sequencer,
  which is not thread proof. This feature is controlled by
  the parameter \ref locking.
*/

#define RTMIDI_CLASSNAME "AlsaSequencer"
template <int locking=1>
class AlsaSequencer {
public:
  AlsaSequencer( )
    : seq( 0 )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
    }
  }

  AlsaSequencer( const std::string& n )
    : seq( 0 ), name( n )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
    }
    init( );
    {
      scoped_lock<locking> lock( mutex );
      snd_seq_set_client_name( seq, name.c_str( ) );
    }
  }

  ~AlsaSequencer( )
  {
    if ( seq ) {
      scoped_lock<locking> lock( mutex );
      snd_seq_close( seq );
      seq = 0;
    }
    if ( locking ) {
      pthread_mutex_destroy( &mutex );
    }
  }

  int setName( const std::string& n ) {
    /* we don't want to rename the client after opening it. */
    name = n;
    if ( seq ) {
      return snd_seq_set_client_name( seq, name.c_str( ) );
    }
    return 0;
  }

  std::string GetPortName( int client, int port, int flags ) {
    init( );
    snd_seq_client_info_t * cinfo;
    snd_seq_client_info_alloca( &cinfo );
    {
      scoped_lock<locking> lock ( mutex );
      snd_seq_get_any_client_info( seq, client, cinfo );
    }

    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );
    {
      scoped_lock<locking> lock ( mutex );
      snd_seq_get_any_port_info( seq, client, port, pinfo );
    }

    int naming = flags & PortDescriptor::NAMING_MASK;

    std::ostringstream os;
    switch ( naming ) {
    case PortDescriptor::SESSION_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "ALSA:";
      os << client << ":" << port;
      break;
    case PortDescriptor::STORAGE_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "ALSA:";
      os << snd_seq_client_info_get_name( cinfo );
      os << ":";
      os << snd_seq_port_info_get_name( pinfo );
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME )
        os << ";" << client << ":" << port;
      break;
    case PortDescriptor::LONG_NAME:
      os << snd_seq_client_info_get_name( cinfo );
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
        os << " " << client;
      }
      os << ":";
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
        os << port;
      }

      os << " " << snd_seq_port_info_get_name( pinfo );
      if ( flags & PortDescriptor::INCLUDE_API )
        os << " ( ALSA )";
      break;
    case PortDescriptor::SHORT_NAME:
    default:
      os << snd_seq_client_info_get_name( cinfo );
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
        os << " ";
        os << client;
      }
      os << ":" << port;
      if ( flags & PortDescriptor::INCLUDE_API )
        os << " ( ALSA )";

      break;
    }
    return os.str( );
  }

  void setPortName( const std::string& name, int port ) {
    snd_seq_port_info_t * pinfo = NULL;
    int error;
    snd_seq_port_info_alloca( &pinfo );
    if ( pinfo == NULL ) {
      throw RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port info structure." ),
                          Error::MEMORY_ERROR );
    }
    if ( ( error = snd_seq_get_port_info( seq, port, pinfo ) ) < 0 ) {
      throw RTMIDI_ERROR1( gettext_noopt( "Could not get ALSA port information: %s" ),
                           Error::DRIVER_ERROR,
                           snd_strerror( error ) );
    }
    snd_seq_port_info_set_name( pinfo, name.c_str( ) );
    if ( ( error = snd_seq_set_port_info( seq, port, pinfo ) ) ) {
      throw RTMIDI_ERROR1( gettext_noopt( "Could not set ALSA port information: %s" ),
                           Error::DRIVER_ERROR,
                           snd_strerror( error ) );
    }
  }

  int getPortCapabilities( int client, int port ) {
    init( );
    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );
    {
      scoped_lock<locking> lock ( mutex );
      snd_seq_get_any_port_info( seq, client, port, pinfo );
    }
    unsigned int caps = snd_seq_port_info_get_capability( pinfo );
    int retval = ( caps & ( SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ ) )?
      PortDescriptor::INPUT : 0;
    if ( caps & ( SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE ) )
      retval |= PortDescriptor::OUTPUT;
    return retval;
  }

  int getNextClient( snd_seq_client_info_t * cinfo ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    return snd_seq_query_next_client ( seq, cinfo );
  }

  int getNextPort( snd_seq_port_info_t * pinfo ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    return snd_seq_query_next_port ( seq, pinfo );
  }

  int createPort ( snd_seq_port_info_t * pinfo ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    return snd_seq_create_port( seq, pinfo );
  }

  void deletePort( int port ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    snd_seq_delete_port( seq, port );
  }

  snd_seq_port_subscribe_t * connectPorts( const snd_seq_addr_t& from,
                                           const snd_seq_addr_t& to,
                                           bool real_time ) {
    init( );
    snd_seq_port_subscribe_t * subscription;

    if ( snd_seq_port_subscribe_malloc( &subscription ) < 0 ) {
      throw RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                          Error::DRIVER_ERROR );
      return 0;
    }
    snd_seq_port_subscribe_set_sender( subscription, &from );
    snd_seq_port_subscribe_set_dest( subscription, &to );
    if ( real_time ) {
      snd_seq_port_subscribe_set_time_update( subscription, 1 );
      snd_seq_port_subscribe_set_time_real( subscription, 1 );
    }
    {
      scoped_lock<locking> lock ( mutex );
      if ( snd_seq_subscribe_port( seq, subscription ) ) {
        snd_seq_port_subscribe_free( subscription );
        subscription = 0;
        throw RTMIDI_ERROR( gettext_noopt( "Error making ALSA port connection." ),
                            Error::DRIVER_ERROR );
        return 0;
      }
    }
    return subscription;
  }

  void closePort( snd_seq_port_subscribe_t * subscription ) {
    init( );
    scoped_lock<locking> lock( mutex );
    snd_seq_unsubscribe_port( seq, subscription );
  }

  void startQueue( int queue_id ) {
    init( );
    scoped_lock<locking> lock( mutex );
    snd_seq_start_queue( seq, queue_id, NULL );
    snd_seq_drain_output( seq );
  }

  /*! Use AlsaSequencer like a C pointer.
    \note This function breaks the design to control thread safety
    by the selection of the \ref locking parameter to the class.
    It should be removed as soon as possible in order ensure the
    thread policy that has been intended by creating this class.
  */
  operator snd_seq_t * ( )
  {
    return seq;
  }
public:
  pthread_mutex_t mutex;
  snd_seq_t * seq;
  std::string name;


  snd_seq_client_info_t * GetClient( int id ) {
    init( );
    snd_seq_client_info_t * cinfo;
    scoped_lock<locking> lock( mutex );
    snd_seq_get_any_client_info( seq, id, cinfo );
    return cinfo;
  }

  void init( )
  {
    init ( seq );
  }

  void init( snd_seq_t *& s )
  {
    if ( s ) return;
    {
      scoped_lock<locking> lock( mutex );
      int result = snd_seq_open( &s, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK );
      if ( result < 0 ) {
        switch ( result ) {
        case -ENOENT: // /dev/snd/seq does not exist
          // Error numbers are defined to be positive
        case -EACCES: // /dev/snd/seq cannot be opened
          throw RTMIDI_ERROR( snd_strerror( result ),
                              Error::NO_DEVICES_FOUND );
          return;
        default:
          std::cerr << __FILE__ << ":" << __LINE__
                    << ": Got unhandled error number " << result << std::endl;
          throw RTMIDI_ERROR( snd_strerror( result ),
                              Error::DRIVER_ERROR );
          return;
        }
      }
      snd_seq_set_client_name( seq, name.c_str( ) );
    }
  }
};
#undef RTMIDI_CLASSNAME
typedef AlsaSequencer<1> LockingAlsaSequencer;
typedef AlsaSequencer<0> NonLockingAlsaSequencer;

#define RTMIDI_CLASSNAME "AlsaPortDescriptor"
struct AlsaPortDescriptor
  : public PortDescriptor,
    public snd_seq_addr_t
{
  MidiApi * api;
  static LockingAlsaSequencer seq;
  AlsaPortDescriptor( const std::string& name )
    : api( 0 ), clientName( name )
  {
    client = 0;
    port = 0;
  }
  AlsaPortDescriptor( int c, int p, const std::string& name )
    : api( 0 ), clientName( name )
  {
    client = c;
    port = p;
    seq.setName( name );
  }
  AlsaPortDescriptor( snd_seq_addr_t& other,
                      const std::string& name )
    : snd_seq_addr_t( other ),
      clientName( name ) {
    seq.setName( name );
  }
  ~AlsaPortDescriptor( ) {}
  InternalMidiApi * getInputApi( ) const;
  InternalMidiApi * getOutputApi( ) const;

  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq.GetPortName( client, port, flags );
  }

  const std::string& getClientName( ) {
    return clientName;
  }

  int getCapabilities( ) const {
    if ( !client ) return 0;
    return seq.getPortCapabilities( client, port );
  }

  virtual bool operator == ( const PortDescriptor& o ) {
    const AlsaPortDescriptor * desc = dynamic_cast<const AlsaPortDescriptor*>( &o );
    if ( !desc ) return false;
    return client == desc->client && port == desc->port;
  }
  static PortList getPortList( int capabilities, const std::string& clientName );
public:
  std::string clientName;
};

LockingAlsaSequencer AlsaPortDescriptor :: seq;



PortList AlsaPortDescriptor :: getPortList( int capabilities, const std::string& clientName )
{
  PortList list;
  snd_seq_client_info_t * cinfo;
  snd_seq_port_info_t * pinfo;
  int client;
  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  snd_seq_client_info_set_client( cinfo, -1 );
  while ( seq.getNextClient( cinfo ) >= 0 ) {
    client = snd_seq_client_info_get_client( cinfo );
    // ignore default device ( it is included in the following results again )
    if ( client == 0 ) continue;
    // Reset query info
    snd_seq_port_info_set_client( pinfo, client );
    snd_seq_port_info_set_port( pinfo, -1 );
    while ( seq.getNextPort( pinfo ) >= 0 ) {
      unsigned int atyp = snd_seq_port_info_get_type( pinfo );
      // otherwise we get ports without any
      if ( !( capabilities & UNLIMITED ) &&
           !( atyp & SND_SEQ_PORT_TYPE_MIDI_GENERIC ) &&
           !( atyp & SND_SEQ_PORT_TYPE_SYNTH )
           ) continue;
      unsigned int caps = snd_seq_port_info_get_capability( pinfo );
      if ( capabilities & INPUT ) {
        /* we need both READ and SUBS_READ */
        if ( ( caps & ( SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ ) )
             != ( SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ ) )
          continue;
      }
      if ( capabilities & OUTPUT ) {
        /* we need both WRITE and SUBS_WRITE */
        if ( ( caps & ( SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE ) )
             != ( SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE ) )
          continue;
      }
      list.push_back( Pointer<PortDescriptor>(
                                              new AlsaPortDescriptor( client, snd_seq_port_info_get_port( pinfo ), clientName ) ) );
    }
  }
  return list;
}
#undef RTMIDI_CLASSNAME


/*! A structure to hold variables related to the ALSA API
  implementation.

  \note After all sequencer handling is covered by the \ref
  AlsaSequencer class, we should make seq to be a pointer in order
  to allow a common client implementation.
*/
#define RTMIDI_CLASSNAME "AlsaMidiData"

struct AlsaMidiData: public AlsaPortDescriptor {
  /*
    AlsaMidiData( )
    : seq( )
    {
    init( );
    }
  */
  AlsaMidiData( const std::string& clientName )
    : AlsaPortDescriptor( clientName ),
      seq( clientName )
  {
    init( );
  }
  ~AlsaMidiData( )
  {
    try {
      if ( local.client && local.port )
        deletePort( );
    } catch ( const Error& e ) {
      // we don't have access to the error handler
      e.printMessage( );
    }
  }
  void init ( ) {
    local.port = 0;
    local.client = 0;
    port = -1;
    subscription = 0;
    coder = 0;
    dummy_thread_id = pthread_self( );
    thread = dummy_thread_id;
    queue_id = -1;
    trigger_fds[0] = -1;
    trigger_fds[1] = -1;
    lastTime.tv_sec = 0;
    lastTime.tv_nsec = 0;
  }
  snd_seq_addr_t local; /*!< Our port and client id. If client = 0 ( default ) this means we didn't aquire a port so far. */
  NonLockingAlsaSequencer seq;
  //  unsigned int portNum;
  snd_seq_port_subscribe_t * subscription;
  snd_midi_event_t * coder;
  std::array<unsigned char, 32> buffer;
  pthread_t thread;
  pthread_t dummy_thread_id;
  snd_seq_real_time_t lastTime;
  int queue_id; // an input queue is needed to get timestamped events
  int trigger_fds[2];

  void setRemote( const AlsaPortDescriptor * remote ) {
    port = remote->port;
    client = remote->client;
  }
  void connectPorts( const snd_seq_addr_t& from,
                     const snd_seq_addr_t& to,
                     bool real_time ) {
    subscription = seq.connectPorts( from, to, real_time );
  }

  /**
   * Do the base work of creating ports. In order to avoid name clashes
   * we have the name in the first place and the capabilities in the secound.
   *
   * \param portName Name of the port to be created.
   * \param alsaCapabilities ALSA bitmask that explains what the port can do.
   *
   * \return error code
   * \retval 0 if everything is ok
   */
  int createPort( int alsaCapabilities,
                  const std::string& portName ) {
    if ( subscription ) {
      api->error( RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                                Error::DRIVER_ERROR ) );
      return -99;
    }

    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );

    snd_seq_port_info_set_client( pinfo, 0 );
    snd_seq_port_info_set_port( pinfo, 0 );
    snd_seq_port_info_set_capability( pinfo,
                                      alsaCapabilities );
    snd_seq_port_info_set_type( pinfo,
                                SND_SEQ_PORT_TYPE_MIDI_GENERIC |
                                SND_SEQ_PORT_TYPE_APPLICATION );
    snd_seq_port_info_set_midi_channels( pinfo, 16 );
#ifndef AVOID_TIMESTAMPING
    snd_seq_port_info_set_timestamping( pinfo, 1 );
    snd_seq_port_info_set_timestamp_real( pinfo, 1 );
    snd_seq_port_info_set_timestamp_queue( pinfo, queue_id );
#endif
    snd_seq_port_info_set_name( pinfo, portName.c_str( ) );
    int createok = seq.createPort( pinfo );

    if ( createok < 0 ) {
      api->error( RTMIDI_ERROR( "ALSA error while creating input port.",
                                Error::DRIVER_ERROR ) );
      return createok;
    }

    local.client = snd_seq_port_info_get_client( pinfo );
    local.port = snd_seq_port_info_get_port( pinfo );
    return 0;
  }

  void deletePort( ) {
    seq.deletePort( local.port );
    local.client = 0;
    local.port = 0;
  }

  void closePort( ) {
    seq.closePort( subscription );
    snd_seq_port_subscribe_free( subscription );
    subscription = 0;
  }

  void setName( const std::string& name ) {
    seq.setPortName( name, local.port );
  }

  void setClientName( const std::string& name ) {
    seq.setName( name );
  }

  bool startQueue( void * userdata );

  long alsa2Midi( const snd_seq_event_t * event,
                  unsigned char * buffer,
                  long size ) {
    return snd_midi_event_decode( coder,
                                  buffer,
                                  size,
                                  event );
  }
};
#undef RTMIDI_CLASSNAME


#define PORT_TYPE( pinfo, bits ) ( ( snd_seq_port_info_get_capability( pinfo ) & ( bits ) ) == ( bits ) )

//*********************************************************************//
// API: LINUX ALSA
// Class Definitions: MidiInAlsa
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiInAlsa"
class MidiInAlsa: public AlsaMidiData,
                  public InternalMidiApi {
public:
  typedef AlsaMidiData base;
  typedef InternalMidiApi api;
  MidiInAlsa( const std::string& clientName );
  ~MidiInAlsa( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::LINUX_ALSA; };
  bool hasVirtualPorts( ) const { return true; }
  void openPort( unsigned int portNumber, const std::string& portName );
  void openVirtualPort( const std::string& portName );
  void openPort( const PortDescriptor& port, const std::string& portName );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& portName ) {
    AlsaMidiData::setClientName( portName );
  }
  void setPortName( const std::string& portName ) {
    AlsaMidiData::setName( portName );
  }
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
public:
  static void * alsaMidiHandler( void * ptr ) throw( );
  void initialize( );
  void doCallback( const snd_seq_event_t * event,
                   MidiMessage& message );

  /**
   * Decode and deliver a chunk of a system exclusive message.
   *
   * \param event ALSA event to be processed
   * \param old_size Number of bytes already stored in the message
   * \param message Message to be delvered to the application
   *
   * \return Number of bytes in an unfinished the message. Otherwise 0.
   */
  size_t doSysEx( snd_seq_event_t * event,
                  size_t old_size,
                  MidiMessage& message );

  bool doAlsaEvent( snd_seq_event_t * event,
                    MidiMessage& message );

  friend class AlsaMidiData; // for registering the callback
};

inline InternalMidiApi * AlsaPortDescriptor :: getInputApi( ) const {
  if ( getCapabilities( ) & INPUT )
    return new MidiInAlsa( clientName );
  else
    return 0;
}

inline InternalMidiApi * AlsaPortDescriptor :: getOutputApi( ) const {
  if ( getCapabilities( ) & OUTPUT )
    return new MidiOutAlsa( clientName );
  else
    return 0;
}

inline bool AlsaMidiData :: startQueue( void * userdata ) {
  // Start the input queue
#ifndef AVOID_TIMESTAMPING
  seq.startQueue( queue_id );
#endif
  // Start our MIDI input thread.
  pthread_attr_t attr;
  pthread_attr_init( &attr );
  pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
  pthread_attr_setschedpolicy( &attr, SCHED_OTHER );

  int err = pthread_create( &thread, &attr, MidiInAlsa::alsaMidiHandler, userdata );
  pthread_attr_destroy( &attr );
  if ( err ) {
    closePort( );
    api->error( RTMIDI_ERROR( gettext_noopt( "Error starting MIDI input thread!" ),
                              Error::THREAD_ERROR ) );
    return false;
  }
  return true;
}

// static function:
void * MidiInAlsa :: alsaMidiHandler( void * ptr ) throw( )
{
  MidiInAlsa * data = static_cast<MidiInAlsa *> ( ptr );

  bool doDecode = false;
  size_t old_size = 0 ;
  MidiMessage message;
  int poll_fd_count;
  struct pollfd * poll_fds;

  snd_seq_event_t * ev;
  int result;
  result = snd_midi_event_new( 0, &data->coder );
  if ( result < 0 ) {
    data->doInput = false;
    try {
      data->error( RTMIDI_ERROR( rtmidi_gettext( "Error initializing MIDI event parser." ),
                                 Error::WARNING ) );
    } catch ( Error& e ) {
      // don't bother ALSA with an unhandled exception
    }
    return 0;
  }

  snd_midi_event_init( data->coder );
#ifdef RTMIDI_DEBUG
  snd_midi_event_no_status( data->coder, 0 ); // debug running status messages
#else
  snd_midi_event_no_status( data->coder, 1 ); // suppress running status messages
#endif

  poll_fd_count = snd_seq_poll_descriptors_count( data->seq, POLLIN ) + 1;
  poll_fds = (struct pollfd*) alloca( poll_fd_count * sizeof( struct pollfd ) );
  snd_seq_poll_descriptors( data->seq, poll_fds + 1, poll_fd_count - 1, POLLIN );
  poll_fds[0].fd = data->trigger_fds[0];
  poll_fds[0].events = POLLIN;

  while ( data->doInput ) {

    if ( snd_seq_event_input_pending( data->seq, 1 ) == 0 ) {
      // No data pending
      if ( poll( poll_fds, poll_fd_count, -1 ) >= 0 ) {
        if ( poll_fds[0].revents & POLLIN ) {
          bool dummy;
          int res = read( poll_fds[0].fd, &dummy, sizeof( dummy ) );
          ( void ) res;
        }
      }
      continue;
    }

    // If here, there should be data.
    result = snd_seq_event_input( data->seq, &ev );
    if ( result == -ENOSPC ) {
      try {
        data->error( RTMIDI_ERROR( rtmidi_gettext( "MIDI input buffer overrun." ),
                                   Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }

      continue;
    }
    else if ( result == -EAGAIN ) {
      try {
        data->error( RTMIDI_ERROR( rtmidi_gettext( "ALSA returned without providing a MIDI event." ),
                                   Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }

      continue;
    }
    else if ( result <= 0 ) {
      try {
        data->error( RTMIDI_ERROR1( rtmidi_gettext( "Unknown MIDI input error.\nThe system reports:\n%s" ),
                                    Error::WARNING,
                                    strerror( -result ) ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }
      continue;
    }

    // This is a bit weird, but we now have to decode an ALSA MIDI
    // event ( back ) into MIDI bytes. We'll ignore non-MIDI types.
    // TODO: provide an event based API

    doDecode = false;
    switch ( ev->type ) {

      // ignore management data
    case SND_SEQ_EVENT_PORT_SUBSCRIBED:
#if defined( __RTMIDI_DEBUG__ )
      std::cerr << "MidiInAlsa::alsaMidiHandler: port connection made!\n";
      std::cerr << "sender = " << ( int ) ev->data.connect.sender.client << ":"
                << ( int ) ev->data.connect.sender.port
                << ", dest = " << ( int ) ev->data.connect.dest.client << ":"
                << ( int ) ev->data.connect.dest.port
                << std::endl;
#endif
      break;

    case SND_SEQ_EVENT_PORT_UNSUBSCRIBED:
#if defined( __RTMIDI_DEBUG__ )
      std::cerr << "MidiInAlsa::alsaMidiHandler: port connection has closed!\n";
      std::cerr << "sender = " << ( int ) ev->data.connect.sender.client << ":"
                << ( int ) ev->data.connect.sender.port
                << ", dest = " << ( int ) ev->data.connect.dest.client << ":"
                << ( int ) ev->data.connect.dest.port
                << std::endl;
#endif
      break;

    case SND_SEQ_EVENT_QFRAME: // MIDI time code
    case SND_SEQ_EVENT_TICK: // 0xF9 ... MIDI timing tick
    case SND_SEQ_EVENT_CLOCK: // 0xF8 ... MIDI timing ( clock ) tick
      if ( !( data->ignoreFlags & IGNORE_TIME ) ) doDecode = true;
      break;

    case SND_SEQ_EVENT_SENSING: // Active sensing
      if ( !( data->ignoreFlags & IGNORE_SENSING ) ) doDecode = true;
      break;

    case SND_SEQ_EVENT_SYSEX:
      if ( ( data->ignoreFlags & IGNORE_SYSEX ) ) break;
      // decode message directly into the buffer

      // The ALSA sequencer has a maximum buffer size for MIDI sysex
      // events of 256 bytes. If a device sends sysex messages larger
      // than this, they are segmented into 256 byte chunks. So,
      // we'll watch for this and concatenate sysex chunks into a
      // single sysex message if necessary.
      try {
        old_size = data->doSysEx( ev,
                                  old_size,
                                  message );
      } catch ( std::bad_alloc& e ) {
        doDecode = false;
        try {
          data->error( RTMIDI_ERROR( rtmidi_gettext( "Error resizing buffer memory." ),
                                     Error::WARNING ) );
        } catch ( Error& e ) {
          // don't bother ALSA with an unhandled exception
        }
      }
      doDecode = false;
      break;

    default:
      doDecode = true;
    }

    if ( doDecode ) {
      if ( data->doAlsaEvent( ev, message ) )
        old_size = 0; // stop decoding SysEx.
    }

    snd_seq_free_event( ev );
  }

  snd_midi_event_free( data->coder );
  data->coder = 0;
  data->thread = data->dummy_thread_id;
  return 0;
}

MidiInAlsa :: MidiInAlsa( const std::string& clientName )
  : AlsaMidiData ( clientName ),
    InternalMidiApi( )
{
  MidiInAlsa::initialize( );
}

MidiInAlsa :: ~MidiInAlsa( )
{
  // Close a connection if it exists.
  MidiInAlsa::closePort( );

  // Shutdown the input thread.
  if ( doInput ) {
    doInput = false;
    int res = write( trigger_fds[1], &doInput, sizeof( doInput ) );
    ( void ) res;
    if ( !pthread_equal( thread, dummy_thread_id ) )
      pthread_join( thread, NULL );
  }

  // Cleanup.
  // TODO: Merge with AlsaMidiApi
  try {
    if ( local.client && local.port )
      deletePort( );
  } catch ( const Error& e ) {
    // we don't have access to the error handler
    e.printMessage( );
  }

#ifndef AVOID_TIMESTAMPING
  snd_seq_free_queue( seq, queue_id );
  queue_id = -1;
#endif
  close ( trigger_fds[0] );
  close ( trigger_fds[1] );
}

inline void MidiInAlsa :: initialize( )
{

  // Save our api-specific connection information.

  if ( pipe( trigger_fds ) == -1 ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating pipe objects." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Create the input queue
#ifndef AVOID_TIMESTAMPING
  queue_id = snd_seq_alloc_named_queue( seq, "Midi Queue" );
  // Set arbitrary tempo ( mm=100 ) and resolution ( 240 )
  snd_seq_queue_tempo_t * qtempo;
  snd_seq_queue_tempo_alloca( &qtempo );
  snd_seq_queue_tempo_set_tempo( qtempo, 600000 );
  snd_seq_queue_tempo_set_ppq( qtempo, 240 );
  snd_seq_set_queue_tempo( seq, queue_id, qtempo );
  snd_seq_drain_output( seq );
#endif
}

// helper functions to avoid compiler warnings
template<class T>
bool is_negative( const T& x ) {
  return x < 0;
}

template<>
bool is_negative<unsigned int>( const unsigned int& ) {
  return false;
}
template<>
bool is_negative<unsigned long>( const unsigned long& ) {
  return false;
}

inline __attribute__( ( always_inline ) )
void MidiInAlsa :: doCallback( const snd_seq_event_t * event,
                               MidiMessage& message ) {

  // Perform the carry for the later subtraction by updating y.
  snd_seq_real_time_t x( event->time.time );
  snd_seq_real_time_t y( lastTime );

  // normalize x
  x.tv_sec += x.tv_nsec/1000000000;
  x.tv_nsec = x.tv_nsec%1000000000;
  while ( is_negative( x.tv_nsec ) ) {
    x.tv_sec --;
    x.tv_nsec += 1000000000;
  }
  lastTime = x;


  if ( firstMessage == true ) {
    // y may not be normalised, but is ignored
    message.timeStamp = 0.0;
    firstMessage = false;
  } else {
    // y is normalized, both, x.tv_nsec and y.tv_nsec
    // are between 0 and 1000000000

    // Calculate the time stamp:

    // Method 1: Use the system time.
    //( void )gettimeofday( &tv, ( struct timezone * )NULL );
    //time = ( tv.tv_sec * 1000000 ) + tv.tv_usec;

    // Method 2: Use the ALSA sequencer event time data.
    // ( thanks to Pedro Lopez-Cabanillas! ).

    if ( x.tv_nsec < y.tv_nsec ) {
      --x.tv_sec;
      x.tv_nsec += ( 1000000000 - y.tv_nsec );
    } else {
      x.tv_nsec -= y.tv_nsec;
    }
    x.tv_sec -= y.tv_sec;

    // Compute the time difference.
    double time = x.tv_sec + x.tv_nsec * 1e-9;


    message.timeStamp = time;
  }
  lastTime = event->time.time;
  if ( userCallback )
    userCallback->rtmidi_midi_in( message.timeStamp, message.bytes );
  else {
    // As long as we haven't reached our queue size limit, push the message.
    if ( !queue.push( message ) ) {
      try {
        error( RTMIDI_ERROR( rtmidi_gettext( "Error: Message queue limit reached." ),
                             Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }
    }
  }
}

/**
 * Decode and deliver a chunk of a system exclusive message.
 *
 * \param event ALSA event to be processed
 * \param old_size Number of bytes already stored in the message
 * \param message Message to be delvered to the application
 *
 * \return Number of bytes in an unfinished the message. Otherwise 0.
 */
inline __attribute__( ( always_inline ) )
size_t MidiInAlsa :: doSysEx( snd_seq_event_t * event,
                              size_t old_size,
                              MidiMessage& message )
{
  message.bytes.resize( old_size + event->data.ext.len );
  long nBytes = alsa2Midi( event,
                           &message.bytes[old_size],
                           message.bytes.size( ) - old_size );
  if ( nBytes > 0 ) {
    old_size += nBytes;
    if ( message.bytes[old_size - 1] == 0xF7 ) {
      old_size = 0;
      doCallback( event, message );
    }
  }
  return old_size;
}

inline __attribute__( ( always_inline ) )
bool MidiInAlsa :: doAlsaEvent( snd_seq_event_t * event,
                                MidiMessage& message ) {
  // we don't have lengths information so we need a
  // secound buffer
  long nBytes = alsa2Midi( event,
                           buffer.data( ),
                           buffer.size( ) );
  if ( nBytes > 0 ) {
    message.bytes.assign( buffer.data( ), buffer.data( ) + nBytes );
    doCallback( event, message );
    return true;
  } else {
#if defined( __RTMIDI_DEBUG__ )
    try {
      error( RTMIDI_ERROR( rtmidi_gettext( "Event parsing error or not a MIDI event." ),
                           Error::WARNING ) );
    } catch ( Error& e ) {
      // don't bother ALSA with an unhandled exception
    }
#endif
    return false;
  }
}


// This function is used to count or get the pinfo structure for a given port number.
unsigned int portInfo( snd_seq_t * seq, snd_seq_port_info_t * pinfo, unsigned int type, int portNumber )
{
  snd_seq_client_info_t * cinfo;
  int client;
  int count = 0;
  snd_seq_client_info_alloca( &cinfo );

  snd_seq_client_info_set_client( cinfo, -1 );
  while ( snd_seq_query_next_client( seq, cinfo ) >= 0 ) {
    client = snd_seq_client_info_get_client( cinfo );
    if ( client == 0 ) continue;
    // Reset query info
    snd_seq_port_info_set_client( pinfo, client );
    snd_seq_port_info_set_port( pinfo, -1 );
    while ( snd_seq_query_next_port( seq, pinfo ) >= 0 ) {
      unsigned int atyp = snd_seq_port_info_get_type( pinfo );
      if ( ( ( atyp & SND_SEQ_PORT_TYPE_MIDI_GENERIC ) == 0 ) &&
           ( ( atyp & SND_SEQ_PORT_TYPE_SYNTH ) == 0 ) ) continue;
      unsigned int caps = snd_seq_port_info_get_capability( pinfo );
      if ( ( caps & type ) != type ) continue;
      if ( count == portNumber ) return 1;
      ++count;
    }
  }

  // If a negative portNumber was used, return the port count.
  if ( portNumber < 0 ) return count;
  return 0;
}

unsigned int MidiInAlsa :: getPortCount( )
{
  snd_seq_port_info_t * pinfo;
  snd_seq_port_info_alloca(& pinfo );

  return portInfo( seq, pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, -1 );
}

std::string MidiInAlsa :: getPortName( unsigned int portNumber )
{
  snd_seq_client_info_t * cinfo;
  snd_seq_port_info_t * pinfo;
  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  std::string stringName;
  if ( portInfo( seq, pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, ( int ) portNumber ) ) {
    int cnum = snd_seq_port_info_get_client( pinfo );
    snd_seq_get_any_client_info( seq, cnum, cinfo );
    std::ostringstream os;
    os << snd_seq_client_info_get_name( cinfo );
    os << ":";
    os << snd_seq_port_info_get_name( pinfo );
    os << " "; // These lines added to make sure devices are listed
    os << snd_seq_port_info_get_client( pinfo ); // with full portnames added to ensure individual device names
    os << ":";
    os << snd_seq_port_info_get_port( pinfo );
    stringName = os.str( );
    return stringName;
  }

  // If we get here, we didn't find a match.
  error( RTMIDI_ERROR( gettext_noopt( "Error looking for port name." ),
                       Error::WARNING ) );
  return stringName;
}

void MidiInAlsa :: openPort( unsigned int portNumber, const std::string& portName )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  unsigned int nSrc = this->getPortCount( );
  if ( nSrc < 1 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI input sources found." ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  snd_seq_port_info_t * src_pinfo;
  snd_seq_port_info_alloca( &src_pinfo );
  if ( portInfo( seq, src_pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, ( int ) portNumber ) == 0 ) {
    std::ostringstream ost;
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER,
                          portNumber ) );
    return;
  }

  snd_seq_addr_t sender, receiver;
  sender.client = snd_seq_port_info_get_client( src_pinfo );
  sender.port = snd_seq_port_info_get_port( src_pinfo );

  snd_seq_port_info_t * pinfo;
  snd_seq_port_info_alloca( &pinfo );
  if ( !local.client ) {
    snd_seq_port_info_set_client( pinfo, 0 );
    snd_seq_port_info_set_port( pinfo, 0 );
    snd_seq_port_info_set_capability( pinfo,
                                      SND_SEQ_PORT_CAP_WRITE |
                                      SND_SEQ_PORT_CAP_SUBS_WRITE );
    snd_seq_port_info_set_type( pinfo,
                                SND_SEQ_PORT_TYPE_MIDI_GENERIC |
                                SND_SEQ_PORT_TYPE_APPLICATION );
    snd_seq_port_info_set_midi_channels( pinfo, 16 );
#ifndef AVOID_TIMESTAMPING
    snd_seq_port_info_set_timestamping( pinfo, 1 );
    snd_seq_port_info_set_timestamp_real( pinfo, 1 );
    snd_seq_port_info_set_timestamp_queue( pinfo, queue_id );
#endif
    snd_seq_port_info_set_name( pinfo, portName.c_str( ) );
    int createok = snd_seq_create_port( seq, pinfo );

    if ( createok < 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA input port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
    local.port = snd_seq_port_info_get_port( pinfo );
    local.client = snd_seq_port_info_get_client( pinfo );
  }

  receiver = local;

  if ( !subscription ) {
    // Make subscription
    if ( snd_seq_port_subscribe_malloc( &subscription ) < 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
    snd_seq_port_subscribe_set_sender( subscription, &sender );
    snd_seq_port_subscribe_set_dest( subscription, &receiver );
    if ( snd_seq_subscribe_port( seq, subscription ) ) {
      snd_seq_port_subscribe_free( subscription );
      subscription = 0;
      error( RTMIDI_ERROR( gettext_noopt( "Error making ALSA port connection." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
  }

  if ( doInput == false ) {
    // Start the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_start_queue( seq, queue_id, NULL );
    snd_seq_drain_output( seq );
#endif
    // Start our MIDI input thread.
    pthread_attr_t attr;
    pthread_attr_init( &attr );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
    pthread_attr_setschedpolicy( &attr, SCHED_OTHER );

    doInput = true;
    int err = pthread_create( &thread, &attr, alsaMidiHandler, this );
    pthread_attr_destroy( &attr );
    if ( err ) {
      snd_seq_unsubscribe_port( seq, subscription );
      snd_seq_port_subscribe_free( subscription );
      subscription = 0;
      doInput = false;
      error( RTMIDI_ERROR( gettext_noopt( "Error starting MIDI input thread!" ),
                           Error::THREAD_ERROR ) );
      return;
    }
  }

  connected_ = true;
}

void MidiInAlsa :: openPort( const PortDescriptor& port,
                             const std::string& portName )
{
  const AlsaPortDescriptor * remote = dynamic_cast<const AlsaPortDescriptor *>( &port );

  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( subscription ) {
    error( RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                         Error::DRIVER_ERROR ) );
    return;
  }
  if ( !remote ) {
    error( RTMIDI_ERROR( gettext_noopt( "ALSA has been instructed to open a non-ALSA MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  try {
    if ( !local.client )
      createPort ( SND_SEQ_PORT_CAP_WRITE
                   | SND_SEQ_PORT_CAP_SUBS_WRITE,
                   portName );
    setRemote( remote );
    connectPorts( *remote,
                  local,
                  false );


    if ( doInput == false ) {
      doInput
        = startQueue( this );
    }

    connected_ = true;
  } catch ( Error& e ) {
    error( e );
  }
}

Pointer<PortDescriptor> MidiInAlsa :: getDescriptor( bool isLocal )
{
  try {
    if ( isLocal ) {
      if ( local.client ) {
        return Pointer<PortDescriptor>( new AlsaPortDescriptor( local, getClientName( ) ) );
      }
    } else {
      if ( client ) {
        return Pointer<PortDescriptor>( new AlsaPortDescriptor( *this, getClientName( ) ) );
      }
    }
  } catch ( Error & e ) {
    error ( e );
  }
  return NULL;
}
PortList MidiInAlsa :: getPortList( int capabilities )
{
  try {
    return AlsaPortDescriptor::getPortList( capabilities | PortDescriptor::INPUT,
                                            getClientName( ) );
  } catch ( Error& e ) {
    error ( e );
    return PortList( );
  }
}



void MidiInAlsa :: openVirtualPort( const std::string& portName )
{
  if ( !local.client ) {
    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );
    snd_seq_port_info_set_capability( pinfo,
                                      SND_SEQ_PORT_CAP_WRITE |
                                      SND_SEQ_PORT_CAP_SUBS_WRITE );
    snd_seq_port_info_set_type( pinfo,
                                SND_SEQ_PORT_TYPE_MIDI_GENERIC |
                                SND_SEQ_PORT_TYPE_APPLICATION );
    snd_seq_port_info_set_midi_channels( pinfo, 16 );
#ifndef AVOID_TIMESTAMPING
    snd_seq_port_info_set_timestamping( pinfo, 1 );
    snd_seq_port_info_set_timestamp_real( pinfo, 1 );
    snd_seq_port_info_set_timestamp_queue( pinfo, queue_id );
#endif
    snd_seq_port_info_set_name( pinfo, portName.c_str( ) );
    int createok = snd_seq_create_port( seq, pinfo );

    if ( createok < 0 ) {
      errorString_ = "MidiInAlsa::openVirtualPort: ";
      error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA virtual port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
    local.port = snd_seq_port_info_get_port( pinfo );
    local.client = snd_seq_port_info_get_client( pinfo );
  }

  if ( doInput == false ) {
    // Wait for old thread to stop, if still running
    if ( !pthread_equal( thread, dummy_thread_id ) )
      pthread_join( thread, NULL );

    // Start the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_start_queue( seq, queue_id, NULL );
    snd_seq_drain_output( seq );
#endif
    // Start our MIDI input thread.
    pthread_attr_t attr;
    pthread_attr_init( &attr );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
    pthread_attr_setschedpolicy( &attr, SCHED_OTHER );

    doInput = true;
    int err = pthread_create( &thread, &attr, alsaMidiHandler, this );
    pthread_attr_destroy( &attr );
    if ( err ) {
      if ( subscription ) {
        snd_seq_unsubscribe_port( seq, subscription );
        snd_seq_port_subscribe_free( subscription );
        subscription = 0;
      }
      doInput = false;
      error( RTMIDI_ERROR( gettext_noopt( "Error starting MIDI input thread!" ),
                           Error::THREAD_ERROR ) );
      return;
    }
  }
}

void MidiInAlsa :: closePort( void )
{
  if ( connected_ ) {
    if ( subscription ) {
      snd_seq_unsubscribe_port( seq, subscription );
      snd_seq_port_subscribe_free( subscription );
      subscription = 0;
    }
    // Stop the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_stop_queue( seq, queue_id, NULL );
    snd_seq_drain_output( seq );
#endif
    connected_ = false;
  }

  // Stop thread to avoid triggering the callback, while the port is intended to be closed
  if ( doInput ) {
    doInput = false;
    int res = write( trigger_fds[1], &doInput, sizeof( doInput ) );
    ( void ) res;
    if ( !pthread_equal( thread, dummy_thread_id ) )
      pthread_join( thread, NULL );
  }
}


#undef RTMIDI_CLASSNAME

//*********************************************************************//
// API: LINUX ALSA
// Class Definitions: MidiOutAlsa
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiOutAlsa"
MidiOutAlsa :: MidiOutAlsa( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiOutAlsa::initialize( clientName );
}

MidiOutAlsa :: ~MidiOutAlsa( )
{
  // Close a connection if it exists.
  MidiOutAlsa::closePort( );

  // Cleanup.
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  if ( data->local.client > 0 ) {
    snd_seq_delete_port( data->seq, data->local.port );
    data->local.port = -1;
  }
  if ( data->coder ) {
    snd_midi_event_free( data->coder );
    data->coder = 0;
  }
  delete data;
}

void MidiOutAlsa :: initialize( const std::string& clientName )
{
#if 0
  // Set up the ALSA sequencer client.
  snd_seq_t * seq;
  int result1 = snd_seq_open( &seq, "default", SND_SEQ_OPEN_OUTPUT, SND_SEQ_NONBLOCK );
  if ( result1 < 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA sequencer client object." ),
                         Error::DRIVER_ERROR, errorString_ ) );
    return;
  }

  // Set client name.
  snd_seq_set_client_name( seq, clientName.c_str( ) );
#endif

  // Save our api-specific connection information.
  AlsaMidiData * data = new AlsaMidiData( clientName );
  apiData_ = (void *) data;
  // data->seq = seq;
  // data->portNum = -1;

  int result = snd_midi_event_new( data->buffer.size( ), &data->coder );
  if ( result < 0 ) {
    delete data;
    error( RTMIDI_ERROR( gettext_noopt( "Error initializing MIDI event parser." ),
                         Error::DRIVER_ERROR ) );
    return;
  }
  snd_midi_event_init( data->coder );
}

unsigned int MidiOutAlsa :: getPortCount( )
{
  snd_seq_port_info_t * pinfo;
  snd_seq_port_info_alloca( &pinfo );

  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  return portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, -1 );
}

std::string MidiOutAlsa :: getPortName( unsigned int portNumber )
{
  snd_seq_client_info_t * cinfo;
  snd_seq_port_info_t * pinfo;
  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  std::string stringName;
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, ( int ) portNumber ) ) {
    int cnum = snd_seq_port_info_get_client( pinfo );
    snd_seq_get_any_client_info( data->seq, cnum, cinfo );
    std::ostringstream os;
    os << snd_seq_client_info_get_name( cinfo );
    os << ":";
    os << snd_seq_port_info_get_name( pinfo );
    os << " "; // These lines added to make sure devices are listed
    os << snd_seq_port_info_get_client( pinfo ); // with full portnames added to ensure individual device names
    os << ":";
    os << snd_seq_port_info_get_port( pinfo );
    stringName = os.str( );
    return stringName;
  }

  // If we get here, we didn't find a match.
  errorString_ = "MidiOutAlsa::getPortName: ";
  error( RTMIDI_ERROR( gettext_noopt( "Error looking for port name." ),
                       Error::WARNING ) );
  return stringName;
}

void MidiOutAlsa :: openPort( unsigned int portNumber, const std::string& portName )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  unsigned int nSrc = this->getPortCount( );
  if ( nSrc < 1 ) {
    errorString_ = "MidiOutAlsa::openPort: !";
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI output sinks found." ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  snd_seq_port_info_t * pinfo;
  snd_seq_port_info_alloca( &pinfo );
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, ( int ) portNumber ) == 0 ) {
    std::ostringstream ost;
    ost << "MidiOutAlsa::openPort: ";
    errorString_ = ost.str( );
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return;
  }

  data->client = snd_seq_port_info_get_client( pinfo );
  data->port = snd_seq_port_info_get_port( pinfo );
  data->local.client = snd_seq_client_id( data->seq );

  if ( !data->local.client ) {
    int port = snd_seq_create_simple_port( data->seq, portName.c_str( ),
                                           SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
                                           SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION );
    if ( port < 0 ) {
      errorString_ = "MidiOutAlsa::openPort: ";
      error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA output port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }

    data->local.port = port;
  }

  // Make subscription
  if ( snd_seq_port_subscribe_malloc( &data->subscription ) < 0 ) {
    snd_seq_port_subscribe_free( data->subscription );
    data->subscription = 0;
    error( RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                         Error::DRIVER_ERROR ) );
    return;
  }
  snd_seq_port_subscribe_set_sender( data->subscription, data );
  snd_seq_port_subscribe_set_dest( data->subscription, &data->local );
  snd_seq_port_subscribe_set_time_update( data->subscription, 1 );
  snd_seq_port_subscribe_set_time_real( data->subscription, 1 );
  if ( snd_seq_subscribe_port( data->seq, data->subscription ) ) {
    snd_seq_port_subscribe_free( data->subscription );
    data->subscription = 0;
    error( RTMIDI_ERROR( gettext_noopt( "Error making ALSA port connection." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  connected_ = true;
}

void MidiOutAlsa :: closePort( void )
{
  if ( connected_ ) {
    AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
    snd_seq_unsubscribe_port( data->seq, data->subscription );
    snd_seq_port_subscribe_free( data->subscription );
    data->subscription = 0;
    connected_ = false;
  }
}

void MidiOutAlsa :: setClientName( const std::string& clientName )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  data->setClientName( clientName );
}

void MidiOutAlsa :: setPortName( const std::string& portName )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  data->setName( portName );
}

void MidiOutAlsa :: openVirtualPort( const std::string& portName )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  if ( !data->local.client ) {
    int port = snd_seq_create_simple_port( data->seq, portName.c_str( ),
                                           SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
                                           SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION );

    if ( port < 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA virtual port." ),
                           Error::DRIVER_ERROR ) );
    }
    data->local.port = port;
    data->local.client = snd_seq_client_id( data->seq );
  }
}

void MidiOutAlsa :: sendMessage( const unsigned char * message, size_t size )
{
  long result;
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  if ( size > data->buffer.size( ) ) {
    result = snd_midi_event_resize_buffer ( data->coder, size );
    if ( result != 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "ALSA error resizing MIDI event buffer." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
  }

  snd_seq_event_t ev;
  snd_seq_ev_clear( &ev );
  snd_seq_ev_set_source( &ev, data->local.port );
  snd_seq_ev_set_subs( &ev );
  snd_seq_ev_set_direct( &ev );

  // In case there are more messages in the stream we send everything
  while ( size && ( result = snd_midi_event_encode( data->coder,
                                                    message,
                                                    size, &ev ) ) > 0 ) {
    // Send the event.
    if ( snd_seq_event_output( data->seq, &ev ) < 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error sending MIDI message to port." ),
                           Error::WARNING ) );
      return;
    }
    snd_seq_drain_output( data->seq );
    if ( size < (size_t) result ) {
      error( RTMIDI_ERROR( gettext_noopt( "ALSA consumed more bytes than availlable." ),
                           Error::WARNING ) );
      return;
    }
    message += result;
    size -= result;
  }
}

void MidiOutAlsa :: openPort( const PortDescriptor& port,
                              const std::string& portName )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  const AlsaPortDescriptor * remote = dynamic_cast<const AlsaPortDescriptor *>( &port );

  if ( !data ) {
    error( RTMIDI_ERROR( gettext_noopt( "Data has not been allocated." ),
                         Error::SYSTEM_ERROR ) );
    return;
  }
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( data->subscription ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error allocating ALSA port subscription." ),
                         Error::DRIVER_ERROR ) );
    return;
  }
  if ( !remote ) {
    error( RTMIDI_ERROR( gettext_noopt( "ALSA has been instructed to open a non-ALSA MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  try {
    if ( !data->local.client )
      data->createPort ( SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                         portName );
    data->setRemote( remote );
    data->connectPorts( data->local, *remote, true );

    connected_ = true;
  } catch ( Error& e ) {
    error ( e );
  }
}
Pointer<PortDescriptor> MidiOutAlsa :: getDescriptor( bool isLocal )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  try {
    if ( isLocal ) {
      if ( data && data->local.client ) {
        return Pointer<PortDescriptor>(
                                       new AlsaPortDescriptor( data->local, data->getClientName( ) ) );
      }
    } else {
      if ( data && data->client ) {
        return Pointer<PortDescriptor>(
                                       new AlsaPortDescriptor( *data, data->getClientName( ) ) );
      }
    }
  } catch ( Error& e ) {
    error( e );
  }
  return NULL;
}
PortList MidiOutAlsa :: getPortList( int capabilities )
{
  AlsaMidiData * data = static_cast<AlsaMidiData *> ( apiData_ );
  try {
    return AlsaPortDescriptor::getPortList( capabilities | PortDescriptor::OUTPUT,
                                            data->getClientName( ) );
  } catch ( Error& e ) {
    return PortList( );
  }
}
#undef RTMIDI_CLASSNAME
#endif // __LINUX_ALSA__


//*********************************************************************//
// API: Windows Multimedia Library ( MM )
//*********************************************************************//

// API information deciphered from:
// - http://msdn.microsoft.com/library/default.asp?url=/library/en-us/multimed/htm/_win32_midi_reference.asp

// Thanks to Jean-Baptiste Berruchon for the sysex code.

#if defined( __WINDOWS_MM__ )

// The Windows MM API is based on the use of a callback function for
// MIDI input. We convert the system specific time stamps to delta
// time values.

RTMIDI_NAMESPACE_END

// Windows MM MIDI header files.
#include <windows.h>
#include <mmsystem.h>
RTMIDI_NAMESPACE_START

// Convert a null-terminated wide string or ANSI-encoded string to UTF-8.
static std::string ConvertToUTF8( const TCHAR * str )
{
  std::string u8str;
  const WCHAR * wstr = L"";
#if defined( UNICODE ) || defined( _UNICODE )
  wstr = str;
#else
  // Convert from ANSI encoding to wide string
  int wlength = MultiByteToWideChar( CP_ACP, 0, str, -1, NULL, 0 );
  std::wstring wstrtemp;
  if ( wlength ) {
    wstrtemp.assign( wlength - 1, 0 );
    MultiByteToWideChar( CP_ACP, 0, str, -1, &wstrtemp[0], wlength );
    wstr = &wstrtemp[0];
  }
#endif
  // Convert from wide string to UTF-8
  int length = WideCharToMultiByte( CP_UTF8, 0, wstr, -1, NULL, 0, NULL, NULL );
  if ( length ) {
    u8str.assign( length - 1, 0 );
    length = WideCharToMultiByte( CP_UTF8, 0, wstr, -1, &u8str[0], length, NULL, NULL );
  }
  return u8str;
}

#define RT_SYSEX_BUFFER_SIZE 1024
#define RT_SYSEX_BUFFER_COUNT 4

/* some header defines UNIQUE_PORT_NAME as a macro */
#ifdef UNIQUE_PORT_NAME
#undef UNIQUE_PORT_NAME
#endif
/*! An abstraction layer for the ALSA sequencer layer. It provides
  the following functionality:
  - dynamic allocation of the sequencer
  - optionallay avoid concurrent access to the ALSA sequencer,
  which is not thread proof. This feature is controlled by
  the parameter \ref locking.
*/

#define RTMIDI_CLASSNAME "WinMMSequencer"
template <int locking=1>
class WinMMSequencer {
public:
  WinMMSequencer( )
    : mutex( 0 ), name( )
  {
    if ( locking ) {
#if 0
      // use mthreads instead
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
#endif
    }
  }

  WinMMSequencer( const std::string& n )
    : name( n )
  {
    if ( locking ) {
#if 0
      // use mthreads instead
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      pthread_mutex_init( &mutex, &attr );
#endif
    }
    init( );
    {
      scoped_lock<locking> lock( mutex );
    }
  }

  ~WinMMSequencer( )
  {
    if ( locking ) {
#if 0
      // use mthreads instead
      pthread_mutex_destroy( &mutex );
#endif
    }
  }

  bool setName( const std::string& n ) {
    /* we don't want to rename the client after opening it. */
    name = n;
    return true;
  }

  std::string getPortName( int port, bool is_input, int flags ) {
    init( );
    int naming = flags & PortDescriptor::NAMING_MASK;
    std::string name;

    unsigned int nDevices = is_input?midiInGetNumDevs( )
      : midiOutGetNumDevs( );
    if ( port < 0 || (unsigned int) port >= nDevices ) {
      throw Error( RTMIDI_ERROR1( gettext_noopt( "The port argument %d is invalid." ),
                                  Error::INVALID_PARAMETER, port ) );
    }

    if ( is_input ) {
      MIDIINCAPS deviceCaps;
      midiInGetDevCaps( port, &deviceCaps, sizeof( MIDIINCAPS ) );

#if defined( UNICODE ) || defined( _UNICODE )
      int length = WideCharToMultiByte( CP_UTF8, 0, deviceCaps.szPname, -1, NULL, 0, NULL, NULL ) - 1;
      name.assign( length, 0 );
      length = WideCharToMultiByte( CP_UTF8,
                                    0,
                                    deviceCaps.szPname,
                                    static_cast<int>( wcslen( deviceCaps.szPname ) ),
                                    &name[0],
                                    length,
                                    NULL,
                                    NULL );
#else
      name = deviceCaps.szPname;
#endif
    } else {
      MIDIOUTCAPS deviceCaps;
      midiOutGetDevCaps( port, &deviceCaps, sizeof( MIDIOUTCAPS ) );

#if defined( UNICODE ) || defined( _UNICODE )
      int length = WideCharToMultiByte( CP_UTF8, 0, deviceCaps.szPname, -1, NULL, 0, NULL, NULL ) - 1;
      name.assign( length, 0 );
      length = WideCharToMultiByte( CP_UTF8,
                                    0,
                                    deviceCaps.szPname,
                                    static_cast<int>( wcslen( deviceCaps.szPname ) ),
                                    &name[0],
                                    length,
                                    NULL,
                                    NULL );
#else
      name = deviceCaps.szPname;
#endif

    }


    std::ostringstream os;
    switch ( naming ) {
    case PortDescriptor::SESSION_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "WinMM:";
      os << port << ":" << name.c_str( );
      break;
    case PortDescriptor::STORAGE_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "WinMM:";
      os << name.c_str( );
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME )
        os << ";" << port;
      break;
    case PortDescriptor::LONG_NAME:
    case PortDescriptor::SHORT_NAME:
    default:
      os << name.c_str( );
      if ( flags & PortDescriptor::UNIQUE_PORT_NAME ) {
        os << " ";
        os << port;
      }
      if ( flags & PortDescriptor::INCLUDE_API )
        os << " ( WinMM )";

      break;
    }
    return os.str( );
  }

public:
  // to keep the API simple
  int mutex;
  std::string name;


  void init( )
  {
    // init ( seq );
  }

};
// typedef WinMMSequencer<1> LockingWinMMSequencer;
typedef WinMMSequencer<0> NonLockingWinMMSequencer;
#undef RTMIDI_CLASSNAME

struct WinMMPortDescriptor : public PortDescriptor
{
  static NonLockingWinMMSequencer seq;
  WinMMPortDescriptor( const std::string& /*cname*/ )
    : name( ), port( 0 ), clientName( name )
  {
  }
  WinMMPortDescriptor( unsigned int p, const std::string& pn, bool i_o, const std::string& n )
    : name( pn ),
      port( p ),
      is_input( i_o ),
      clientName( n )
  {
  }
  ~WinMMPortDescriptor( ) {}
  InternalMidiApi * getInputApi( ) const {
    if ( is_input )
      return new MidiInWinMM( clientName );
    else
      return 0;
  }
  InternalMidiApi * getOutputApi( ) const {
    if ( !is_input )
      return new MidiOutWinMM( clientName );
    else
      return 0;
  }
  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq.getPortName( port, is_input, flags );
  }

  const std::string& getClientName( ) const {
    return clientName;
  }
  int getCapabilities( ) const {
    return is_input ? INPUT : OUTPUT;
  }

  int getCapabilities( ) {
    const WinMMPortDescriptor * self = this;
    return self->getCapabilities( );
  }

  bool is_valid( ) const {
    if ( is_input ) {
      if ( midiInGetNumDevs( ) <= port ) {
        return false;
      }
    } else {
      if ( midiOutGetNumDevs( ) <= port ) {
        return false;
      }
    }
    return seq.getPortName( port, is_input, PortDescriptor::STORAGE_PATH )
      == name;
  }

  void setRemote( const WinMMPortDescriptor * remote ) {
    port = remote->port;
    name = remote->name;
    is_input = remote->is_input;
  }


  unsigned int getPortNumber( ) const { return port; }

  virtual bool operator == ( const PortDescriptor& o ) {
    const WinMMPortDescriptor * desc = dynamic_cast<const WinMMPortDescriptor *>( &o );
    if ( !desc ) return false;
    return is_input == desc->is_input && port == desc->port;
  }

  static PortList getPortList( int capabilities, const std::string& clientName );
public:
  /* There is no perfect port descriptor available in this API.
     We use the port number and issue an error if the port name has changed
     between the creation of the port descriptor and opening the port. */
  std::string name;
  unsigned int port;
  bool is_input;
  std::string clientName;
};

NonLockingWinMMSequencer WinMMPortDescriptor :: seq;



PortList WinMMPortDescriptor :: getPortList( int capabilities, const std::string& clientName )
{
  PortList list;

  if ( capabilities & INPUT && capabilities & OUTPUT ) return list;

  if ( capabilities & INPUT ) {
    size_t n = midiInGetNumDevs( );
    for ( size_t i = 0 ; i < n ; i++ ) {
      std::string name = seq.getPortName( i, true, PortDescriptor::STORAGE_PATH );
      list.push_back( Pointer<PortDescriptor>(
                                              new WinMMPortDescriptor( i, name, true, clientName ) ) );
    }
  } else {
    size_t n = midiOutGetNumDevs( );
    for ( size_t i = 0 ; i < n ; i++ ) {
      std::string name = seq.getPortName( i, false, PortDescriptor::STORAGE_PATH );
      list.push_back( Pointer<PortDescriptor>(
                                              new WinMMPortDescriptor( i, name, false, clientName ) ) );
    }
  }
  return list;
}


/*! A structure to hold variables related to the WINMM API
  implementation.

  \note After all sequencer handling is covered by the \ref
  WinMMSequencer class, we should make seq to be a pointer in order
  to allow a common client implementation.
*/

struct WinMidiData : public WinMMPortDescriptor {
  /*
    WinMMMidiData( )
    : seq( )
    {
    init( );
    }
  */
  WinMidiData( const std::string& clientName )
    : WinMMPortDescriptor( clientName ) {}
  ~WinMidiData( ) {}

  HMIDIIN inHandle; // Handle to Midi Input Device
  HMIDIOUT outHandle; // Handle to Midi Output Device
  DWORD lastTime;
  MidiMessage message;
  LPMIDIHDR sysexBuffer[RT_SYSEX_BUFFER_COUNT];
  CRITICAL_SECTION _mutex; // [Patrice] see https://groups.google.com/forum/#!topic/mididev/6OUjHutMpEo
};


//*********************************************************************//
// API: Windows MM
// Class Definitions: MidiInWinMM
//*********************************************************************//
#define RTMIDI_CLASSNAME "WinMMCallbacks"
//! Windows callbacks
/*! In order to avoid including too many header files in RtMidi.h, we use this
 * class to callect all friend functions of Midi*WinMM.
 */
struct WinMMCallbacks {
  static void CALLBACK midiInputCallback( HMIDIIN /*hmin*/,
                                          UINT inputStatus,
                                          DWORD_PTR instancePtr,
                                          DWORD_PTR midiMessage,
                                          DWORD timestamp )
  {
    if ( inputStatus != MIM_DATA && inputStatus != MIM_LONGDATA && inputStatus != MIM_LONGERROR ) return;

    //InternalMidiApi::MidiInData * data = static_cast<InternalMidiApi::MidiInData *> ( instancePtr );
    MidiInWinMM * data = static_cast<MidiInWinMM*> instancePtr;
    WinMidiData * apiData = static_cast<WinMidiData *> ( data->apiData_ );

    // Calculate time stamp.
    if ( data->firstMessage == true ) {
      apiData->message.timeStamp = 0.0;
      data->firstMessage = false;
    }
    else apiData->message.timeStamp = ( double ) ( timestamp - apiData->lastTime ) * 0.001;

    if ( inputStatus == MIM_DATA ) { // Channel or system message

      // Make sure the first byte is a status byte.
      unsigned char status = ( unsigned char ) ( midiMessage & 0x000000FF );
      if ( !( status & 0x80 ) ) return;

      // Determine the number of bytes in the MIDI message.
      unsigned short nBytes = 1;
      if ( status < 0xC0 ) nBytes = 3;
      else if ( status < 0xE0 ) nBytes = 2;
      else if ( status < 0xF0 ) nBytes = 3;
      else if ( status == 0xF1 ) {
        if ( data->ignoreFlags & IGNORE_TIME ) return;
        else nBytes = 2;
      }
      else if ( status == 0xF2 ) nBytes = 3;
      else if ( status == 0xF3 ) nBytes = 2;
      else if ( status == 0xF8 && ( data->ignoreFlags & IGNORE_TIME ) ) {
        // A MIDI timing tick message and we're ignoring it.
        return;
      }
      else if ( status == 0xFE && ( data->ignoreFlags & IGNORE_SENSING ) ) {
        // A MIDI active sensing message and we're ignoring it.
        return;
      }

      // Copy bytes to our MIDI message.
      unsigned char * ptr = static_cast<unsigned char*> (&midiMessage);
      for ( int i=0; i<nBytes; ++i ) apiData->message.bytes.push_back( *ptr++ );
    }
    else { // Sysex message ( MIM_LONGDATA or MIM_LONGERROR )
      MIDIHDR * sysex = static_cast<MIDIHDR *>(midiMessage);
      if ( !( data->ignoreFlags & IGNORE_SYSEX ) && inputStatus != MIM_LONGERROR ) {
        // Sysex message and we're not ignoring it
        for ( int i=0; i<( int )sysex->dwBytesRecorded; ++i )
          apiData->message.bytes.push_back( sysex->lpData[i] );
      }

      // The WinMM API requires that the sysex buffer be requeued after
      // input of each sysex message. Even if we are ignoring sysex
      // messages, we still need to requeue the buffer in case the user
      // decides to not ignore sysex messages in the future. However,
      // it seems that WinMM calls this function with an empty sysex
      // buffer when an application closes and in this case, we should
      // avoid requeueing it, else the computer suddenly reboots after
      // one or two minutes.
      if ( apiData->sysexBuffer[sysex->dwUser]->dwBytesRecorded > 0 ) {
        //if ( sysex->dwBytesRecorded > 0 ) {
        EnterCriticalSection( &( apiData->_mutex ) );
        MMRESULT result = midiInAddBuffer( apiData->inHandle, apiData->sysexBuffer[sysex->dwUser], sizeof( MIDIHDR ) );
        LeaveCriticalSection( &( apiData->_mutex ) );
        if ( result != MMSYSERR_NOERROR ){
          try {
            data->error( RTMIDI_ERROR( rtmidi_gettext( "Error sending sysex to Midi device." ),
                                       Error::WARNING ) );
          } catch ( Error& e ) {
            // don't bother WinMM with an unhandled exception
          }
        }

        if ( data->ignoreFlags & IGNORE_SYSEX ) return;
      }
      else return;
    }

    // Save the time of the last non-filtered message
    apiData->lastTime = timestamp;

    if ( data->userCallback ) {
      data->userCallback->rtmidi_midi_in( apiData->message.timeStamp, apiData->message.bytes );
    }
    else {
      // As long as we haven't reached our queue size limit, push the message.
      if ( !data->queue.push( apiData->message ) ) {
        try {
          data->error( RTMIDI_ERROR( rtmidi_gettext( "Error: Message queue limit reached." ),
                                     Error::WARNING ) );
        } catch ( Error& e ) {
          // don't bother WinMM with an unhandled exception
        }
      }
    }

    // Clear the vector for the next input message.
    apiData->message.bytes.clear( );
  }
};
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "MidiInWinMM"
MidiInWinMM :: MidiInWinMM( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiInWinMM::initialize( clientName );
}

MidiInWinMM :: ~MidiInWinMM( )
{
  // Close a connection if it exists.
  MidiInWinMM::closePort( );

  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  DeleteCriticalSection( &( data->_mutex ) );

  // Cleanup.
  delete data;
}

void MidiInWinMM :: initialize( const std::string& clientName )
{
  // We'll issue a warning here if no devices are available but not
  // throw an error since the user can plugin something later.
  unsigned int nDevices = midiInGetNumDevs( );
  if ( nDevices == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI input devices currently available." ),
                         Error::WARNING ) );
  }

  // Save our api-specific connection information.
  WinMidiData * data = new WinMidiData( clientName );
  apiData_ = (void *) data;
  data->message.bytes.clear( ); // needs to be empty for first input message

  if ( !InitializeCriticalSectionAndSpinCount( &( data->_mutex ), 0x00000400 ) ) {
    error( RTMIDI_ERROR( gettext_noopt( "Failed to initialize a critical section." ),
                         Error::WARNING ) );
  }
}

void MidiInWinMM :: openPort( unsigned int portNumber, const std::string& /*portName*/ )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  unsigned int nDevices = midiInGetNumDevs( );
  if ( nDevices == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI input sources found." ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  if ( portNumber >= nDevices ) {
    std::ostringstream ost;
    ost << "MidiInWinMM::openPort: ";
    errorString_ = ost.str( );
    error( RTMIDI_ERROR1( gettext_noopt( "the 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return;
  }

  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  MMRESULT result = midiInOpen( &data->inHandle,
                                portNumber,
                                ( DWORD_PTR )&WinMMCallbacks::midiInputCallback,
                                ( DWORD_PTR )this,
                                CALLBACK_FUNCTION );
  if ( result != MMSYSERR_NOERROR ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating Windows MM MIDI input port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // Allocate and init the sysex buffers.
  for ( int i=0; i<RT_SYSEX_BUFFER_COUNT; ++i ) {
    data->sysexBuffer[i] = static_cast<MIDIHDR*> (new char[ sizeof( MIDIHDR ) ]);
    data->sysexBuffer[i]->lpData = new char[ RT_SYSEX_BUFFER_SIZE ];
    data->sysexBuffer[i]->dwBufferLength = RT_SYSEX_BUFFER_SIZE;
    data->sysexBuffer[i]->dwUser = i; // We use the dwUser parameter as buffer indicator
    data->sysexBuffer[i]->dwFlags = 0;

    result = midiInPrepareHeader( data->inHandle, data->sysexBuffer[i], sizeof( MIDIHDR ) );
    if ( result != MMSYSERR_NOERROR ) {
      midiInClose( data->inHandle );
      data->inHandle = 0;
      error( RTMIDI_ERROR( gettext_noopt( "Error initializing data for Windows MM MIDI input port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }

    // Register the buffer.
    result = midiInAddBuffer( data->inHandle, data->sysexBuffer[i], sizeof( MIDIHDR ) );
    if ( result != MMSYSERR_NOERROR ) {
      midiInClose( data->inHandle );
      data->inHandle = 0;
      error( RTMIDI_ERROR( gettext_noopt( "Could not register the input buffer for Windows MM MIDI input port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
  }

  result = midiInStart( data->inHandle );
  if ( result != MMSYSERR_NOERROR ) {
    midiInClose( data->inHandle );
    data->inHandle = 0;
    error( RTMIDI_ERROR( gettext_noopt( "Error starting Windows MM MIDI input port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  connected_ = true;
}

void MidiInWinMM :: openVirtualPort( const std::string& /*portName*/ )
{
  // This function cannot be implemented for the Windows MM MIDI API.
  error( RTMIDI_ERROR( gettext_noopt( "Virtual ports are not available Windows Multimedia MIDI API." ),
                       Error::WARNING ) );
}

void MidiInWinMM :: openPort( const PortDescriptor& p, const std::string& portName ) {
  const WinMMPortDescriptor * port = dynamic_cast <const WinMMPortDescriptor * >( &p );
  if ( !port ) {
    error( RTMIDI_ERROR( gettext_noopt( "Windows Multimedia ( WinMM ) has been instructed to open a non-WinMM MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "We are overwriting an existing connection. This is probably a programming error." ),
                         Error::WARNING ) );
    return;
  }
  if ( port->getCapabilities( ) != PortDescriptor :: INPUT ) {
    error( RTMIDI_ERROR( gettext_noopt( "Trying to open a non-input port as input MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  // there is a possible race condition between opening the port and
  // reordering of ports so we must check whether we opened the right port.
  try {
    openPort( port->getPortNumber( ), portName );
  } catch ( Error& e ) {
    error( e );
  }
  if ( !port->is_valid( ) ) {
    closePort( );
    error ( RTMIDI_ERROR( gettext_noopt( "Some change in the arrangement of MIDI input ports invalidated the port descriptor." ),
                          Error::DRIVER_ERROR ) );
    return;
  }
  connected_ = true;
}

Pointer<PortDescriptor> MidiInWinMM :: getDescriptor( bool isLocal )
{
  if ( isLocal || !connected_ ) return 0;
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  if ( !data ) return 0;
  UINT devid;
  switch ( midiInGetID( data->inHandle, &devid ) ) {
  case MMSYSERR_INVALHANDLE:
    error ( RTMIDI_ERROR( gettext_noopt( "The handle is invalid. Did you disconnect the device?" ),
                          Error::DRIVER_ERROR ) );
    return 0;
  case MMSYSERR_NODRIVER:
    error ( RTMIDI_ERROR( gettext_noopt( "The system has no driver for our handle :-( . Did you disconnect the device?" ),
                          Error::DRIVER_ERROR ) );
    return 0;
  case MMSYSERR_NOMEM:
    error ( RTMIDI_ERROR( gettext_noopt( "Out of memory." ),
                          Error::DRIVER_ERROR ) );
    return 0;
  }
  WinMMPortDescriptor * retval = NULL;
  try {
    retval = new WinMMPortDescriptor( devid, getPortName( devid ), true, data->getClientName( ) );
  } catch ( Error& e ) {
    try {
      error( e );
    } catch ( ... ) {
      if ( retval ) delete retval;
      throw;
    }
  }
  return Pointer<PortDescriptor>( retval );

}

PortList MidiInWinMM :: getPortList( int capabilities )
{
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  if ( !data || capabilities != PortDescriptor::INPUT ) return PortList( );
  try {
    return WinMMPortDescriptor::getPortList( PortDescriptor::INPUT, data->getClientName( ) );
  } catch ( Error& e ) {
    error( e );
    return PortList( );
  }
}


void MidiInWinMM :: closePort( void )
{
  if ( connected_ ) {
    WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
    EnterCriticalSection( &( data->_mutex ) );
    midiInReset( data->inHandle );
    midiInStop( data->inHandle );

    for ( int i=0; i<RT_SYSEX_BUFFER_COUNT; ++i ) {
      int result = midiInUnprepareHeader( data->inHandle, data->sysexBuffer[i], sizeof( MIDIHDR ) );
      delete [] data->sysexBuffer[i]->lpData;
      delete [] data->sysexBuffer[i];
      if ( result != MMSYSERR_NOERROR ) {
        midiInClose( data->inHandle );
        data->inHandle = 0;
        error( RTMIDI_ERROR( gettext_noopt( "Error closing Windows MM MIDI input port." ),
                             Error::DRIVER_ERROR ) );
        return;
      }
    }

    midiInClose( data->inHandle );
    data->inHandle = 0;
    connected_ = false;
    LeaveCriticalSection( &( data->_mutex ) );
  }
}

void MidiInWinMM :: setClientName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting the client name is not supported by Windows MM." ),
                       Error::WARNING ) );
}

void MidiInWinMM :: setPortName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting the port name is not supported by Windows MM." ),
                       Error::WARNING ) );
}

unsigned int MidiInWinMM :: getPortCount( )
{
  return midiInGetNumDevs( );
}

std::string MidiInWinMM :: getPortName( unsigned int portNumber )
{
  std::string stringName;
  unsigned int nDevices = midiInGetNumDevs( );
  if ( portNumber >= nDevices ) {
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::WARNING, portNumber ) );
    return stringName;
  }

  MIDIINCAPS deviceCaps;
  midiInGetDevCaps( portNumber, &deviceCaps, sizeof( MIDIINCAPS ) );
  stringName = ConvertToUTF8( deviceCaps.szPname );

  // Next lines added to add the portNumber to the name so that
  // the device's names are sure to be listed with individual names
  // even when they have the same brand name
#ifndef RTMIDI_DO_NOT_ENSURE_UNIQUE_PORTNAMES
  std::ostringstream os;
  os << " ";
  os << portNumber;
  stringName += os.str( );
#endif

  return stringName;
}
#undef RTMIDI_CLASSNAME



//*********************************************************************//
// API: Windows MM
// Class Definitions: MidiOutWinMM
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiOutWinMM"
MidiOutWinMM :: MidiOutWinMM( const std::string& clientName )
  : InternalMidiApi( )
{
  MidiOutWinMM::initialize( clientName );
}

MidiOutWinMM :: ~MidiOutWinMM( )
{
  // Close a connection if it exists.
  MidiOutWinMM::closePort( );

  // Cleanup.
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  delete data;
}

void MidiOutWinMM :: initialize( const std::string& clientName )
{
  // We'll issue a warning here if no devices are available but not
  // throw an error since the user can plug something in later.
  unsigned int nDevices = midiOutGetNumDevs( );
  if ( nDevices == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI output devices currently available." ),
                         Error::WARNING ) );
  }

  // Save our api-specific connection information.
  WinMidiData * data = new WinMidiData( clientName );
  apiData_ = (void *) data;
}

unsigned int MidiOutWinMM :: getPortCount( )
{
  return midiOutGetNumDevs( );
}

std::string MidiOutWinMM :: getPortName( unsigned int portNumber )
{
  std::string stringName;
  unsigned int nDevices = midiOutGetNumDevs( );
  if ( portNumber >= nDevices ) {
    std::ostringstream ost;
    ost << "MidiOutWinMM::getPortName: ";
    errorString_ = ost.str( );
    error( RTMIDI_ERROR( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                         Error::WARNING ) );
    return stringName;
  }

  MIDIOUTCAPS deviceCaps;
  midiOutGetDevCaps( portNumber, &deviceCaps, sizeof( MIDIOUTCAPS ) );
  stringName = ConvertToUTF8( deviceCaps.szPname );

  // Next lines added to add the portNumber to the name so that
  // the device's names are sure to be listed with individual names
  // even when they have the same brand name
  std::ostringstream os;
#ifndef RTMIDI_DO_NOT_ENSURE_UNIQUE_PORTNAMES
  os << " ";
  os << portNumber;
  stringName += os.str( );
#endif

  return stringName;
}


void MidiOutWinMM :: openPort( unsigned int portNumber, const std::string& /*portName*/ )
{
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }

  unsigned int nDevices = midiOutGetNumDevs( );
  if ( nDevices < 1 ) {
    error( RTMIDI_ERROR( gettext_noopt( "No MIDI output destinations found!" ),
                         Error::NO_DEVICES_FOUND ) );
    return;
  }

  if ( portNumber >= nDevices ) {
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return;
  }

  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  MMRESULT result = midiOutOpen( &data->outHandle,
                                 portNumber,
                                 ( DWORD )NULL,
                                 ( DWORD )NULL,
                                 CALLBACK_NULL );
  if ( result != MMSYSERR_NOERROR ) {
    error( RTMIDI_ERROR( gettext_noopt( "Error creating Windows MM MIDI output port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  connected_ = true;
}

void MidiOutWinMM :: closePort( void )
{
  if ( connected_ ) {
    WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
    midiOutReset( data->outHandle );
    midiOutClose( data->outHandle );
    data->outHandle = 0;
    connected_ = false;
  }
}

void MidiOutWinMM :: setClientName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting the client name is not supported by Windows MM." ),
                       Error::WARNING ) );
}

void MidiOutWinMM :: setPortName ( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting the port name is not supported by Windows MM." ),
                       Error::WARNING ) );
}

void MidiOutWinMM :: openVirtualPort( const std::string& /*portName*/ )
{
  // This function cannot be implemented for the Windows MM MIDI API.
  error( RTMIDI_ERROR( gettext_noopt( "Virtual ports are not available Windows Multimedia MIDI API." ),
                       Error::WARNING ) );
}


void MidiOutWinMM :: openPort( const PortDescriptor& p, const std::string& portName ) {
  const WinMMPortDescriptor * port = dynamic_cast <const WinMMPortDescriptor * >( &p );
  if ( !port ) {
    error( RTMIDI_ERROR( gettext_noopt( "Windows Multimedia ( WinMM ) has been instructed to open a non-WinMM MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }
  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( port->getCapabilities( ) != PortDescriptor::OUTPUT ) {
    error( RTMIDI_ERROR( gettext_noopt( "The port descriptor cannot be used to open an output port." ),
                         Error::DRIVER_ERROR ) );
    return;
  }

  // there is a possible race condition between opening the port and
  // reordering of ports so we must check whether we opened the right port.
  try {
    openPort( port->getPortNumber( ), portName );
  } catch ( Error& e ) {
    error( e );
  }
  if ( !port->is_valid( ) ) {
    closePort( );
    error ( RTMIDI_ERROR( gettext_noopt( "Some change in the arrangement of MIDI input ports invalidated the port descriptor." ),
                          Error::DRIVER_ERROR ) );
    return;
  }
  connected_ = true;
}

Pointer<PortDescriptor> MidiOutWinMM :: getDescriptor( bool isLocal )
{
  if ( isLocal || !connected_ ) return 0;
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  if ( !data ) return 0;
  UINT devid;
  switch ( midiOutGetID( data->outHandle, &devid ) ) {
  case MMSYSERR_INVALHANDLE:
    error ( RTMIDI_ERROR( gettext_noopt( "The internal handle is invalid. Did you disconnect the device?" ),
                          Error::DRIVER_ERROR ) );
    return 0;
  case MMSYSERR_NODRIVER:
    error ( RTMIDI_ERROR( gettext_noopt( "The system has no driver for our handle :-( . Did you disconnect the device?" ),
                          Error::DRIVER_ERROR ) );
    return 0;
  case MMSYSERR_NOMEM:
    error ( RTMIDI_ERROR( gettext_noopt( "Out of memory." ),
                          Error::DRIVER_ERROR ) );
    return 0;
  }
  return Pointer<PortDescriptor>(
                                 new WinMMPortDescriptor( devid, getPortName( devid ), true, data->getClientName( ) ) );

}

PortList MidiOutWinMM :: getPortList( int capabilities )
{
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  if ( !data || capabilities != PortDescriptor::OUTPUT ) return PortList( );
  try {
    return WinMMPortDescriptor::getPortList( PortDescriptor::OUTPUT, data->getClientName( ) );
  } catch ( Error& e ) {
    error( e );
    return PortList( );
  }
}


void MidiOutWinMM :: sendMessage( const unsigned char * message, size_t size )
{
  if ( !connected_ ) return;

  if ( size == 0 ) {
    error( RTMIDI_ERROR( gettext_noopt( "Message argument is empty." ),
                         Error::WARNING ) );
    return;
  }

  MMRESULT result;
  WinMidiData * data = static_cast<WinMidiData *> ( apiData_ );
  if ( message[0] == 0xF0 ) { // Sysex message

    // Allocate buffer for sysex data.
    char * buffer = (char *) malloc( size );
    if ( buffer == NULL ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error while allocating sysex message memory." ),
                           Error::MEMORY_ERROR ) );
      return;
    }

    // Copy data to buffer.
    for ( unsigned int i=0; i<size; ++i ) buffer[i] = message[i];

    // Create and prepare MIDIHDR structure.
    MIDIHDR sysex;
    sysex.lpData = ( LPSTR ) buffer;
    sysex.dwBufferLength = size;
    sysex.dwFlags = 0;
    result = midiOutPrepareHeader( data->outHandle, &sysex, sizeof( MIDIHDR ) );
    if ( result != MMSYSERR_NOERROR ) {
      free( buffer );
      error( RTMIDI_ERROR( gettext_noopt( "Error preparing sysex header." ),
                           Error::DRIVER_ERROR ) );
      return;
    }

    // Send the message.
    result = midiOutLongMsg( data->outHandle, &sysex, sizeof( MIDIHDR ) );
    if ( result != MMSYSERR_NOERROR ) {
      free( buffer );
      error( RTMIDI_ERROR( gettext_noopt( "Error sending sysex message." ),
                           Error::DRIVER_ERROR ) );
      return;
    }

    // Unprepare the buffer and MIDIHDR.
    while ( MIDIERR_STILLPLAYING == midiOutUnprepareHeader( data->outHandle, &sysex, sizeof ( MIDIHDR ) ) ) Sleep( 1 );
    free( buffer );
  }
  else { // Channel or system message.

    // Make sure the message size isn't too big.
    if ( size > 3 ) {
      error( RTMIDI_ERROR( gettext_noopt( "Message size is greater than 3 bytes ( and not sysex )." ),
                           Error::WARNING ) );
      return;
    }

    // Pack MIDI bytes into double word.
    DWORD packet;
    unsigned char * ptr = static_cast<unsigned char *> &packet;
    for ( unsigned int i=0; i<size; ++i ) {
      *ptr = message[i];
      ++ptr;
    }

    // Send the message immediately.
    result = midiOutShortMsg( data->outHandle, packet );
    if ( result != MMSYSERR_NOERROR ) {
      error( RTMIDI_ERROR( gettext_noopt( "Error sending MIDI message." ),
                           Error::DRIVER_ERROR ) );
    }
  }
}
#undef RTMIDI_CLASSNAME
#endif // __WINDOWS_MM__


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


struct JackMidi;
class MidiJack;

#define RTMIDI_CLASSNAME "JackSequencer"
template <int locking=1>
class JackSequencer {
public:
  JackSequencer( )
    : client( 0 ), name( ), data( nullptr )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      if ( pthread_mutex_init( &mutex, &attr ) ) {
        throw RTMIDI_ERROR( gettext_noopt( "Could not initialise the JACK client mutex" ),
                            Error::MEMORY_ERROR );
      }
    }
  }

  JackSequencer( const std::string& n, JackMidi& d )
    : client( 0 ), name( n ), data( &d )
  {
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      if ( pthread_mutex_init( &mutex, &attr ) ) {
        throw RTMIDI_ERROR( gettext_noopt( "Could not initialise the JACK client mutex" ),
                            Error::MEMORY_ERROR );
      }
    }
  }

  ~JackSequencer( )
  {
    {
      scoped_lock<locking> lock ( mutex );
      if ( client ) {
        jack_deactivate ( client );
        // the latter doesn't flush the queue
        jack_client_close ( client );
        client = 0;
      }
    }
    if ( locking ) {
      pthread_mutex_destroy( &mutex );
    }
  }

  void init( bool startqueue ) {
    if ( !client ) {
      init( client, startqueue );
      jack_activate( client );
    }
  }

  bool setName( const std::string& n ) {
    /* we don't want to rename the client after opening it. */
    if ( client ) return false;
    name = n;
    return true;
  }

  const char ** getPortList( unsigned long flags ) {
    init( );
    return jack_get_ports( client,
                           NULL,
                           "midi",
                           flags );
  }

  jack_port_t * getPort( const char * name ) {
    init( );
    return jack_port_by_name( client, name );
  }

#if !__UNIX_JACK_HAS_UUID__
  typedef uint64_t jack_uuid_t;
  jack_uuid_t int jack_port_uuid( jack_port_t* ) {
    return 0;
  }
#else
  typedef ::jack_uuid_t jack_uuid_t;
#endif

  std::string getPortName( jack_port_t * port, int flags ) {
    init( );
    int naming = flags & PortDescriptor::NAMING_MASK;

    std::ostringstream os;
    switch ( naming ) {
    case PortDescriptor::SESSION_PATH:
      {
        if ( flags & PortDescriptor::INCLUDE_API )
          os << "JACK:";
        // currently Jackd2 implements a dummy UUID returning zero.
        auto uuid = jack_port_uuid( port );
        if ( uuid )
          os << "UUID:" << std::hex << uuid;
        else
          os << jack_port_name( port );
        break;
      }
    case PortDescriptor::STORAGE_PATH:
      if ( flags & PortDescriptor::INCLUDE_API )
        os << "JACK:";
      os << jack_port_name( port );
      break;
    case PortDescriptor::LONG_NAME:
      os << jack_port_name( port );
      if ( flags & PortDescriptor::INCLUDE_API )
        os << " ( JACK )";
      break;
    case PortDescriptor::SHORT_NAME:
    default:
      os << jack_port_short_name( port );
      if ( flags & PortDescriptor::INCLUDE_API )
        os << " ( JACK )";
      break;
    }
    return os.str( );
  }

  int getPortCapabilities( jack_port_t * port ) {
    if ( !port ) return 0;
    const char * type = jack_port_type( port );
    if ( !type || strcmp( type, JACK_DEFAULT_MIDI_TYPE ) ) return 0;
    int flags = jack_port_flags( port );
    int retval = 0;
    /* a JACK input port is capable of handling output to it and vice versa */
    if ( flags & JackPortIsInput )
      retval |= PortDescriptor::OUTPUT;
    if ( flags & JackPortIsOutput )
      retval |= PortDescriptor::INPUT;

    return retval;
  }


  jack_port_t * createPort ( const std::string& portName, unsigned long portOptions ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    return jack_port_register( client,
                               portName.c_str( ),
                               JACK_DEFAULT_MIDI_TYPE,
                               portOptions,
                               0 );
  }

  int renamePort( jack_port_t * port, const std::string& portName ) {
#ifdef JACK_HAS_PORT_RENAME
    return jack_port_rename( client, port, portName.c_str( ) );
#else
    return jack_port_set_name( port, portName.c_str( ) );
#endif
  }

  void deletePort( jack_port_t * port ) {
    init( );
    scoped_lock<locking> lock ( mutex );
    jack_port_unregister( client, port );
  }

  void connectPorts( jack_port_t * from,
                     jack_port_t * to )
  {
    init( );
    jack_connect( client,
                  jack_port_name( from ),
                  jack_port_name( to ) );
  }

  void connectPorts( const std::string& from,
                     jack_port_t * to )
  {
    init( );
    jack_connect( client,
                  from.c_str( ),
                  jack_port_name( to ) );
  }

  void closePort( jack_port_t * port )
  {
    init( );
    jack_port_disconnect( client, port );
  }


  /*! Use JackSequencer like a C pointer.
    \note This function breaks the design to control thread safety
    by the selection of the \ref locking parameter to the class.
    It should be removed as soon as possible in order ensure the
    thread policy that has been intended by creating this class.
  */
  operator jack_client_t * ( )
  {
    return client;
  }

protected:
  pthread_mutex_t mutex;
  jack_client_t * client;
  std::string name;
  JackMidi * data;


  void init( )
  {
    if ( !client ) {
      init ( client, false );
      jack_activate( client );
    }
  }

  void init( jack_client_t *& c, bool isoutput )
  {
    if ( c ) return;
    {
      scoped_lock<locking> lock( mutex );
      if ( ( c = jack_client_open( name.c_str( ),
                                   JackNoStartServer,
                                   NULL ) ) == 0 ) {
        c = NULL;
        throw RTMIDI_ERROR( gettext_noopt( "Could not connect to JACK server. Is it runnig?" ),
                            Error::NO_DEVICES_FOUND );
        return;
      }

      if ( data )
        jack_set_process_callback( c, Callback, data );
      // don't activate the client as we might want to set further callbacks
    }
  }

  static int Callback ( jack_nframes_t nframes, void * arg );
};
typedef JackSequencer<1> LockingJackSequencer;
typedef JackSequencer<0> NonLockingJackSequencer;
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "JackPortDescriptor"
struct JackPortDescriptor : public PortDescriptor
{
  static LockingJackSequencer seq;
  JackPortDescriptor( const std::string& name )
    : /*api( 0 ), */clientName( name ), port( nullptr )
  {
  }
  JackPortDescriptor( const char * portname, const std::string& name )
    : /* api( 0 ), */clientName( name )
  {
    seq.setName( clientName );
    port = seq.getPort( portname );
  }

  JackPortDescriptor( JackPortDescriptor& other,
                      const std::string& name )
    : /*api( 0 ), */
    clientName( name )
  {
#if 0
    port = other.port;
#else
    // assign port to the permanent sequencer
    port = other.clonePort( seq );
#endif
    seq.setName( clientName );
  }

  ~JackPortDescriptor( )
  {
  }

  MidiApi * getApi ( unsigned int capabilities ) const;

  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq.getPortName( port, flags );
  }

  const std::string& getClientName( ) {
    return clientName;
  }
  int getCapabilities( ) const {
    return seq.getPortCapabilities( port );
  }

  /**
   * Create a copy of the Jack port information that belongs to the
   * given sequencer.
   *
   * \param other JackPortDescriptor
   *
   * \return port information of the same port as we point to, but
   * attached to another JACK sequencer. So that it survives
   * when our sequencer gets destroyed.
   */
  template<int i>
  jack_port_t * clonePort( JackSequencer<i>& seq ) const {
    const char * portName = jack_port_name( port );
    return seq.getPort( portName );
  }


  // TODO: better comparison
  virtual bool operator == ( const PortDescriptor& o ) {
    const JackPortDescriptor * desc = dynamic_cast<const JackPortDescriptor *>( &o );
    if ( !desc ) return false;
    if ( !port ) return false;
    // danger! We must ensure that port always point to seq
    return port == desc->port;
    // in case the above turns out not to work properly:
#if 0
    if ( port == desc->port ) return true;
    return ( seq.getPortName( port, SESSION_PATH ) ==
             desc->seq.getPortName( desc->port, SESSION_PATH ) );
#endif
  }

  static PortList getPortList( int capabilities, const std::string& clientName );

  //operator jack_port_t * ( ) const { return port; }

public:
  std::string clientName;
  jack_port_t * port;

  friend struct JackMidi;

  static long int jackCapabilities( int capabilities ) {
    unsigned long flags = 0;

    if ( capabilities & INPUT ) {
      flags |= JackPortIsOutput;
    }
    if ( capabilities & OUTPUT ) {
      flags |= JackPortIsInput;
    }
    return flags;
  }

  JackPortDescriptor( jack_port_t * other,
                      const std::string& name ): /*api( 0 ), */
    clientName( name )
  {
    if ( !other ) {
      throw RTMIDI_ERROR( gettext_noopt( "Invalid port submitted to JackPortDescriptor( )." ),
                          Error::DRIVER_ERROR );
    }
    // copy the port structure in 2 steps to avoid crashes.
    // Set ourself to the desired structure.
    port = other;
    // clone the structure with our sequencer.
    port = clonePort( seq );
    seq.setName( clientName );
  }
};

LockingJackSequencer JackPortDescriptor :: seq;



PortList JackPortDescriptor :: getPortList( int capabilities, const std::string& clientName )
{
  PortList list;
  const char ** ports = seq.getPortList( jackCapabilities( capabilities ) );
  if ( !ports ) return list;
  for ( const char ** port = ports; *port; port++ ) {
    list.push_back( Pointer<PortDescriptor>(new JackPortDescriptor( *port, clientName ) ) );
  }
  jack_free( ports );
  return list;
}
#undef RTMIDI_CLASSNAME

/*! A structure to hold variables related to the JACK API
  implementation.

  \note After all sequencer handling is covered by the \ref
  JackSequencer class, we should make seq to be a pointer in order
  to allow a common client implementation.
*/

#define RTMIDI_CLASSNAME "JackMidi"
struct JackMidi : public JackPortDescriptor {
  /* signal the JACK process what to do next */
  volatile enum {
                 RUNNING, /*!< keep the client open, flag is owned by the controlling process */
                 CLOSING, /*!< close the current port */
                 DELETING /*!< Delete the client after delivering the contents of the ring buffer */
  } stateflags;
  /*! response/state from the JACK thread. See \ref ProcessOut for details */
  volatile enum {
                 OPEN,
                 CLOSING2,
                 CLOSED,
                 DELETING2,
                 DELETING3
                 /* DELETED is useless as this doesn't exist anymore */
  } state_response;

  jack_port_t * local;
  jack_ringbuffer_t * buffSize;
  jack_ringbuffer_t * buffMessage;
  jack_time_t lastTime;
#ifdef HAVE_SEMAPHORE
  sem_t sem_cleanup;
  sem_t sem_needpost;
#endif
  MidiJack * api;
  /*! Sequencer object
    : This must be deleted _before_ the MIDI data to avoid
    segmentation faults while queued data is still in the ring buffer. */
  NonLockingJackSequencer * seq;

  /*
    JackMidi( )
    : seq( )
    {
    init( );
    }
  */
  JackMidi( const std::string& clientName,
            MidiJack * inputData_ )
    : JackPortDescriptor( clientName ),
      stateflags( RUNNING ),
      state_response( OPEN ),
      local( 0 ),
      buffSize( 0 ),
      buffMessage( 0 ),
      lastTime( 0 ),
      api( inputData_ ),
      seq( new NonLockingJackSequencer( clientName, *this ) )
  {
#ifdef HAVE_SEMAPHORE
    sem_init( &sem_cleanup, 0, 0 );
    sem_init( &sem_needpost, 0, 0 );
#endif
  }

  /**
   * Create output midi data.
   *
   * \param clientName
   *
   * \return
   */
  JackMidi( const std::string& clientName )
    : JackPortDescriptor( clientName ),
      stateflags( RUNNING ),
      state_response( OPEN ),
      local( 0 ),
      buffSize( jack_ringbuffer_create( JACK_RINGBUFFER_SIZE ) ),
      buffMessage( jack_ringbuffer_create( JACK_RINGBUFFER_SIZE ) ),
      lastTime( 0 ),
      api( ),
      seq( new NonLockingJackSequencer( clientName, *this ) )
  {
#ifdef HAVE_SEMAPHORE
    sem_init( &sem_cleanup, 0, 0 );
    sem_init( &sem_needpost, 0, 0 );
#endif
  }


  ~JackMidi( )
  {
    if ( local )
      try {
        deletePort( );
      } catch ( const Error& e ) {
        e.printMessage( std::cerr );
      }
    if ( seq )
      delete seq;
#ifdef HAVE_SEMAPHORE
    sem_destroy( &sem_cleanup );
    sem_destroy( &sem_needpost );
#endif

    if ( buffSize ) {
      jack_ringbuffer_free( buffSize );
      buffSize = 0;
    }
    if ( buffMessage ) {
      jack_ringbuffer_free( buffMessage );
      buffMessage = 0;
    }
  }

  void init( bool isinput ) {
    seq->init( !isinput );
  }

  void setRemote( const JackPortDescriptor& o ) {
    port = o.clonePort( *seq );
  }

  void connectFrom( const JackPortDescriptor& from ) {
    setRemote( from );
    seq->connectPorts( port, local );
  }

  void connectTo( const JackPortDescriptor& to ) {
    setRemote( to );
    seq->connectPorts( local, port );
  }

  int getPortCount( unsigned long jackCapabilities ) {
    int count = 0;
    // connect( );
    if ( !( *( seq ) ) )
      return 0;

    // List of available ports
    const char ** ports = jack_get_ports( *( seq ), NULL, JACK_DEFAULT_MIDI_TYPE, jackCapabilities );

    if ( ports == NULL ) return 0;
    while ( ports[count] != NULL )
      count++;

    free( ports );

    return count;
  }

  std::string getPortName( unsigned int portNumber,
                           unsigned long jackCapabilities )
  {
    std::string retStr( "" );

    //  connect( );

    // List of available ports
    const char ** ports = jack_get_ports( * ( seq ), NULL,
                                          JACK_DEFAULT_MIDI_TYPE, jackCapabilities );

    // Check port validity
    if ( ports == NULL ) {
      throw RTMIDI_ERROR( gettext_noopt( "No ports available." ),
                          Error::WARNING );
      return retStr;
    }

    unsigned int i;
    for ( i=0; i<portNumber && ports[i]; i++ ) {}
    if ( i < portNumber || !ports[portNumber] ) {
      // std::ostringstream ost;
      throw RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                           Error::WARNING, portNumber );
    }
    else retStr.assign( ports[portNumber] );

    jack_free( ports );
    return retStr;
  }

  int openPort( unsigned long jackCapabilities,
                const std::string& portName ) {
    local = seq->createPort( portName, jackCapabilities );
    if ( !local ) {
      throw RTMIDI_ERROR( gettext_noopt( "Error opening JACK port subscription." ),
                          Error::DRIVER_ERROR );
    }
    return 0;
  }

  int ensureOpen( unsigned long jackCapabilities,
                  const std::string& portName ) {
    if ( !local )
      return openPort ( jackCapabilities, portName );
    else return 0;
  }

  int rename( const std::string& portName ) {
    return seq->renamePort( local, portName );
  }


  Pointer<PortDescriptor> getDescriptor( bool isLocal )
  {
    if ( isLocal ) {
      if ( local ) {
        return Pointer<PortDescriptor>( new JackPortDescriptor( local, getClientName( ) ) );
      }
    } else {
      return Pointer<PortDescriptor>( new JackPortDescriptor( *this, getClientName( ) ) );
    }
    return NULL;
  }

  void delayedDeletePort( ) {
    /* Closing the port can be twofold to ensure all data is sent:
       - Use a semaphore to wait for this state
       - Close the port from within ProcessOut
    */
    if ( local == NULL ) return;

#ifdef HAVE_SEMAPHORE
    struct timespec ts;
    if ( clock_gettime( CLOCK_REALTIME, &ts ) != -1 )
      {
        ts.tv_sec += 1; // wait max one second
        sem_post( &sem_needpost );
        sem_timedwait( &sem_cleanup, &ts );
      }

    deletePort( );
#else
    if ( local == NULL || state_response == JackMidi::CLOSED ) return;
    stateflags = JackMidi::CLOSING;
#endif
#if defined( __RTMIDI_DEBUG__ )
    std::cerr << "Closed Port" << std::endl;
#endif
  }

  void request_delete( ) {
    // signal the output callback to delete the data
    // after finishing its job.
    // this can be done twofold:
    // - via signal in ProcessOut
    // - using a semaphore
#ifdef HAVE_SEMAPHORE
    if ( local )
      closePort( );

    // Cleanup
    delete this;
    return;
#else
    stateflags = JackMidi::DELETING;
#endif
  }

  void deletePortIfRequested( ) {
#ifdef HAVE_SEMAPHORE
    if ( !sem_trywait( &sem_needpost ) )
      sem_post( &sem_cleanup );
#else
    switch ( stateflags ) {
    case JackMidi::RUNNING: break;
    case JackMidi::CLOSING:
      if ( state_response != JackMidi::CLOSING2 ) {
        /* output the transferred data */
        state_response = JackMidi::CLOSING2;
        return;
      }
      deletePort( );
      state_response = JackMidi::CLOSED;
      break;

    case JackMidi::DELETING:
#if defined( __RTMIDI_DEBUG__ )
      std::cerr << "deleting port" << std::endl;
#endif
      if ( state_response != JackMidi::DELETING2 ) {
        state_response = JackMidi::DELETING2;
        /* output the transferred data */
        return;
      }

      delete this;
      return;
#if defined( __RTMIDI_DEBUG__ )
      std::cerr << "deleted port" << std::endl;
#endif
      break;
    }
#endif
  }

  void deletePort( ) {
    if ( local == NULL )
      return;

#ifdef HAVE_SEMAPHORE
    struct timespec ts;
    if ( clock_gettime( CLOCK_REALTIME, &ts ) != -1 ) {
      ts.tv_sec += 2; // wait max two seconds
      sem_post( &sem_needpost );
      sem_timedwait( &sem_cleanup, &ts );
    }
#endif

    seq->deletePort( local );
    local = NULL;
  }

  void closePort( ) {
    seq->closePort( local );
    local = NULL;
  }

  operator jack_port_t * ( ) const { return port; }
};
#undef RTMIDI_CLASSNAME

class MidiJack : public InternalMidiApi
{
public:
  MidiJack( const std::string& clientName );
  ~MidiJack( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::UNIX_JACK; };
  unsigned int getCapabilities ( ) throw ( ) {
    return PortDescriptor::INOUTPUT
      | PortDescriptor::VIRTUAL
      | PortDescriptor::UNLIMITED;
  }
  bool hasVirtualPorts( ) const { return true; }
  void openVirtualPort( const std::string& portName,
                        unsigned int capabilities );
  void openPort( const PortDescriptor& port,
                 const std::string& portName,
                 unsigned int capabilities );
#if 0
  void openPort( unsigned int portNumber,
                 const std::string& portName,
                 unsigned int capabilities );
#endif
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
  void sendMessage( const unsigned char * message, size_t size );

public:
  JackMidi * midi;
  void initialize( const std::string& clientName );
};


//*********************************************************************//
// API: JACK
// Class Definitions: MidiJack
//*********************************************************************//

#define RTMIDI_CLASSNAME "JackSequencer"

template <int locking>
int JackSequencer<locking> :: Callback( jack_nframes_t nframes, void * arg )
{
  JackMidi * data = static_cast<JackMidi*>(arg);
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
  return 0;
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "JackPortDescriptor"
MidiApi * JackPortDescriptor :: getApi ( unsigned int capabilities ) const {
  // either input or output or both can be handled by the same API class
  if ( getCapabilities( ) & INOUTPUT ) {
    MidiApi * api = new MidiJack( clientName );
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
    return JackPortDescriptor::getPortList( capabilities,
                                            midi->getClientName( ) );
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

void MidiJack :: setClientName( const std::string& )
{
  error( RTMIDI_ERROR( gettext_noopt( "Setting the client name is not supported by JACK." ),
                       Error::WARNING ) );
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
  midi = new JackMidi( clientName );
  if ( !midi ) {
    error ( RTMIDI_ERROR( gettext_noopt( "Cannot allocate JACK MIDI data object." ),
                          Error::MEMORY_ERROR ) );
    return;
  }
  // init is the last as it may throw an exception
  try {
    midi->init( false );
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

//*********************************************************************//
// API: Common definitons
//*********************************************************************//



#ifdef RTMIDI_GETTEXT
void RTMIDI_DLL_PUBLIC init_rtmidi_gettext( ) {
  static bool initialized = false;
  if ( initialized )
    return;
  bindtextdomain( "rtmidi", LOCALEDIR );
  initialized = true;
}

RTMIDI_DLL_PUBLIC const char * rtmidi_gettext ( const char * s ) {
  init_rtmidi_gettext( );
  return dgettext( "rtmidi", s );
}

#endif

//! The constructor.
Error::Error( const char * message,
              Type type,
              const char * class_name,
              const char * function_name,
              const char * file_name,
              int line_number, ... ) throw( )
  : exception( ),
    classname( class_name ),
    function( function_name ),
    file( file_name ),
    line( line_number ),
    type_( type )
{
#ifdef RTMIDI_GETTEXT
  message = rtmidi_gettext( message );
#endif
  std::va_list args;
  va_start( args, line_number );
  size_t length;
  length = vsnprintf( NULL, 0, message, args );
  if ( length > 0 ) {
    message_.resize( length+1 );
    std::vsnprintf( &( message_[0] ), length+1, message, args );
    message_.resize( length );
  } else {
    const char * fmt = gettext_noopt( "Error formatting the error string:\n'%s'\nFound in %s::%s at \n%s:%d" );
#ifdef RTMIDI_GETTEXT
    fmt = rtmidi_gettext( fmt );
#endif

    length = snprintf( NULL, 0, fmt, message, class_name, function_name, file_name, line );
    if ( length > 0 ) {
      message_.resize( length+1 );
      snprintf( &( message_[0] ), length+1, fmt, message, class_name, function_name, file_name, line );
      message_.resize( length );
    } else {
      const char * msg
        = gettext_noopt( "Error: could not format the error message" );
#ifdef RTMIDI_GETTEXT
      msg = rtmidi_gettext( msg );
#endif
      message_ = msg;
    }
  }
  va_end( args );

}

MidiApi *  Midi::openMidiApi( ApiType api )
{
  try {
    switch ( api ) {
    case rtmidi::UNIX_JACK:
#if defined( __UNIX_JACK__ )
      return new MidiJack( clientName );
#endif
      break;
    case rtmidi::LINUX_ALSA:
#if defined( __LINUX_ALSA__ )
      return new MidiAlsa( clientName );
#endif
      break;
    case rtmidi::WINDOWS_MM:
#if defined( __WINDOWS_MM__ )
      return new MidiWinMM( clientName );
#endif
      break;
    case rtmidi::MACOSX_CORE:
#if defined( __MACOSX_COREMIDI__ )
      return new MidiCore( clientName );
#endif
      break;
    case rtmidi::DUMMY:
#if defined( __RTMIDI_DUMMY__ )
      return new MidiDummy( clientName );
#endif
      break;
    case rtmidi::ALL_API:
    case rtmidi::UNSPECIFIED:
    default:
      break;
    }
  } catch ( const Error& e ) {
    throw;
  }
  return 0;
}

//*********************************************************************//
// Midi Definitions
//*********************************************************************//
std::string Midi :: getVersion( void ) throw( )
{
  return std::string( RTMIDI_VERSION );
}


// This is a compile-time check that rtmidi_num_api_names == RtMidi::NUM_APIS.
// If the build breaks here, check that they match.
template<bool b> class StaticAssert { public: StaticAssert( ) {} };
template<> class StaticAssert<true>{ public: StaticAssert( ) {} };
class StaticAssertions { StaticAssertions( ) {
  StaticAssert<rtmidi_num_api_names == NUM_APIS>( );
}};

void Midi :: getCompiledApi( std::vector<ApiType>& apis, bool preferSystem ) throw( )
{
  apis.clear( );
  apis.reserve( rtmidi_num_compiled_system_apis +
                rtmidi_num_compiled_software_apis +
                rtmidi_num_compiled_other_apis );

  // The order here will control the order of RtMidi's API search in
  // the constructor.

  if ( !preferSystem ) {
    // first check software and network backends
    apis.insert( apis.end( ),
                 rtmidi_compiled_software_apis,
                 rtmidi_compiled_software_apis+rtmidi_num_compiled_software_apis );
  }
  apis.insert( apis.end( ),
               rtmidi_compiled_system_apis,
               rtmidi_compiled_system_apis+rtmidi_num_compiled_system_apis );

  if ( preferSystem ) {
    // if we prefer OS provided backends,
    // we should add the software backends, here.
    apis.insert( apis.end( ),
                 rtmidi_compiled_software_apis,
                 rtmidi_compiled_software_apis+rtmidi_num_compiled_software_apis );
  }

  // pseudo-backends
  apis.insert( apis.end( ),
               rtmidi_compiled_other_apis,
               rtmidi_compiled_other_apis+rtmidi_num_compiled_other_apis );
}


std::string Midi :: getApiName( ApiType api )
{
  if ( api < 0 || api >= NUM_APIS )
    return "";
  return rtmidi_api_names[api].machine;
}

std::string Midi :: getApiDisplayName( ApiType api )
{
  if ( api < 0 || api >= NUM_APIS )
    return "Unknown";
  return rtmidi_gettext( rtmidi_api_names[api].display );
}

ApiType Midi :: getCompiledApiByName( const std::string& name, bool preferSystem )
{
  auto apis = getCompiledApi( preferSystem );
  for ( auto api : apis ) {
    if ( name == rtmidi_api_names[api].machine )
      return api;
  }
  return UNSPECIFIED;
}




void Midi :: error( Error e )
{

  // use the callback if present.
  if ( !apis.empty() ) {
    apis.front()->error( e );
    return;
  }

  if ( e.getType( ) == Error::WARNING ) {
    e.printMessage( );
  }
  else if ( e.getType( ) == Error::DEBUG_WARNING ) {
#if defined( __RTMIDI_DEBUG__ )
    e.printMessage( );
#endif
  }
  else {
    e.printMessage( );
    throw e;
  }
}


//*********************************************************************//
// MidiIn Definitions
//*********************************************************************//
#define RTMIDI_CLASSNAME "Midi"

RTMIDI_DLL_PUBLIC Midi :: Midi( ApiType api,
                                unsigned int capabilities,
                                const std::string& clientName,
                                bool pfsystem )
  : Midi( pfsystem, clientName )
{
  std::vector<ApiType> queryApis;
  // if api == UNSPECIFIED we also collect all APIs, here. But we
  // will keep only one later after some additional tests.
  if ( api == rtmidi::ALL_API || api == rtmidi::UNSPECIFIED ) {
    if ( !queryApis.empty( ) ) {
      throw( RTMIDI_ERROR( gettext_noopt( "No supported MIDI system has been found." ),
                           Error::NO_DEVICES_FOUND ) );
    }

    //std::vector< ApiType > apis;
    getCompiledApi( queryApis, pfsystem );
    for ( unsigned int i=0; i<apis.size( ); i++ ) {
      try {
        MidiApi * rtapi_ = openMidiApi( queryApis[i] );
        if ( rtapi_ ) {
          apis.push_back( MidiApiPtr( rtapi_ ) );
        }
      } catch ( const Error& e ) {
        if ( e.getType( ) != Error::NO_DEVICES_FOUND )
          throw;
      }
    }
    return;
  } else if ( api != rtmidi::UNSPECIFIED ) {
    // Attempt to open the specified API.
    MidiApi * rtapi_ = openMidiApi( api );
    if ( rtapi_ ) {
      apis.emplace_back(rtapi_);
      return;
    }
    if (api == rtmidi::ALL_API ) return;

    // No compiled support for specified API value. Issue a warning
    // and continue as if no API was specified.
    throw RTMIDI_ERROR1( gettext_noopt( "Support for the selected MIDI system %d has not been compiled into the RtMidi library." ),
                         Error::INVALID_PARAMETER,
                         api );
  }

  // Iterate through the compiled APIs and return as soon as we find
  // one with at least one port or we reach the end of the list.
  for ( auto & api : apis ) {
    if ( api->getPortCount( capabilities ) ) {
      MidiApiPtr rtapi_ = api;
      apis.clear();
      apis.push_back(rtapi_);
      return;
    }
  }
  for ( auto & api : apis ) {
    if ( api->hasVirtualPorts( ) ) {
       MidiApiPtr rtapi_ = api;
       queryApis.clear();
       break;
    }
  }
  if (!apis.empty()) {
    apis.erase(++(apis.begin()), apis.end());
  }
}

MidiIn :: ~MidiIn ( ) throw( )
{
}
#undef RTMIDI_CLASSNAME


//*********************************************************************//
// MidiOut Definitions
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiOut"
#if 0
void MidiOut :: openMidiApi( ApiType api )
{
  delete rtapi_;
  rtapi_ = 0;

  try {
    switch ( api ) {
    case rtmidi::UNIX_JACK:
#if defined( __UNIX_JACK__ )
      rtapi_ = new MidiJack( clientName );
#endif
      break;
    case rtmidi::LINUX_ALSA:
#if defined( __LINUX_ALSA__ )
      rtapi_ = new MidiOutAlsa( clientName );
#endif
      break;
    case rtmidi::WINDOWS_MM:
#if defined( __WINDOWS_MM__ )
      rtapi_ = new MidiOutWinMM( clientName );
#endif
      break;
    case rtmidi::MACOSX_CORE:
#if defined( __MACOSX_COREMIDI__ )
      rtapi_ = new MidiOutCore( clientName );
#endif
      break;
    case rtmidi::DUMMY:
#if defined( __RTMIDI_DUMMY__ )
      rtapi_ = new MidiOutDummy( clientName );
#endif
      break;
    case rtmidi::UNSPECIFIED:
    case rtmidi::ALL_API:
    default:
      break;
    }
  } catch ( const Error& e ) {
    rtapi_ = 0;
    throw;
  }

}
MidiApiList MidiOut :: queryApis;

#endif

#if 0
RTMIDI_DLL_PUBLIC MidiOut :: MidiOut( ApiType api, const std::string& clientName, bool pfsystem )
  : Midi( api, PortDescriptor::OUTPUT, clientName, pfsystem )
{
  if ( api == rtmidi::ALL_API ) {
    if ( !queryApis.empty( ) ) {
      rtapi_ = NULL;
      return;
    }

    std::vector< ApiType > apis;
    getCompiledApi( apis );
    for ( unsigned int i=0; i<apis.size( ); i++ ) {
      try {
        openMidiApi( apis[i] );
        if ( rtapi_ ) {
          queryApis.push_back( MidiApiPtr( rtapi_ ) );
          rtapi_ = NULL;
        }
      } catch ( const Error& e ) {
        if ( e.getType( ) != Error::NO_DEVICES_FOUND )
          throw;
      }
    }
    return;
  }

  if ( api != rtmidi::UNSPECIFIED ) {
    // Attempt to open the specified API.
    openMidiApi( api );
    if ( rtapi_ ) return;

    // No compiled support for specified API value. Issue a warning
    // and continue as if no API was specified.
    throw RTMIDI_ERROR1( gettext_noopt( "Support for the selected MIDI system %d has not been compiled into the RtMidi library." ),
                         Error::INVALID_PARAMETER, api );
  }

  // Iterate through the compiled APIs and return as soon as we find
  // one with at least one port or we reach the end of the list.
  std::vector< ApiType > apis;
  getCompiledApi( apis );
  for ( unsigned int i=0; i<apis.size( ); i++ ) {
    switch (apis[i]) {
    case rtmidi::UNSPECIFIED:
    case rtmidi::ALL_API:
      continue;
      break;
    default:
      openMidiApi( apis[i] );
      break;
    }
    if ( rtapi_ && rtapi_->getPortCount( ) ) return;
  }

  // We may reach this point, e.g. if JACK is the only
  // compiled API, but no JACK devices are found.
  throw( RTMIDI_ERROR( gettext_noopt( "No supported MIDI system has been found." ),
                       Error::NO_DEVICES_FOUND ) );
}
#endif

MidiOut :: ~MidiOut( ) throw( )
{
}
#undef RTMIDI_CLASSNAME


MidiApi :: MidiApi( void )
  : connected_( false ),
    firstErrorOccurred_ ( false ),
    errorCallback_( 0 )
{
}

MidiApi :: ~MidiApi( void )
{
}

void MidiApi :: setErrorCallback( ErrorCallback errorCallback, void * userData )
{
  errorCallback_ = new CompatibilityErrorInterface( errorCallback, userData );
}

void MidiApi :: setErrorCallback( ErrorInterface * callback ) {
  errorCallback_ = callback;
}


void MidiApi :: error( Error e )
{
  if ( errorCallback_ ) {

    if ( firstErrorOccurred_ )
      return;

    firstErrorOccurred_ = true;
    std::ostringstream s;
    e.printMessage( s );

    errorCallback_->rtmidi_error( e );
    firstErrorOccurred_ = false;
    return;
  }

  if ( e.getType( ) == Error::WARNING ) {
    e.printMessage( );
  }
  else if ( e.getType( ) == Error::DEBUG_WARNING ) {
#if defined( __RTMIDI_DEBUG__ )
    e.printMessage( );
#endif
  }
  else {
    e.printMessage( );
    throw e;
  }
}

//*********************************************************************//
// Common MidiQueue Definitions
//*********************************************************************//


#define RTMIDI_CLASSNAME "MidiQueue"
unsigned int MidiQueue :: size( unsigned int * __back,
                                unsigned int * __front )
{
  // Access back/front members exactly once and make stack copies for
  // size calculation
  unsigned int _back = back, _front = front, _size;
  if ( _back >= _front )
    _size = _back - _front;
  else
    _size = ringSize - _front + _back;

  // Return copies of back/front so no new and unsynchronized accesses
  // to member variables are needed.
  if ( __back ) *__back = _back;
  if ( __front ) *__front = _front;
  return _size;
}

// As long as we haven't reached our queue size limit, push the message.
bool MidiQueue :: push( const MidiMessage& msg )
{
  // Local stack copies of front/back
  unsigned int _back, _front, _size;

  // Get back/front indexes exactly once and calculate current size
  _size = size( &_back, &_front );

  if ( _size < ringSize-1 )
    {
      ring[_back] = msg;
      back = ( back+1 )%ringSize;
      return true;
    }

  return false;
}

bool MidiQueue :: pop( std::vector<unsigned char>& msg, double& timeStamp )
{
  // Local stack copies of front/back
  unsigned int _back, _front, _size;

  // Get back/front indexes exactly once and calculate current size
  _size = size( &_back, &_front );

  if ( _size == 0 )
    return false;

  // Copy queued message to the vector pointer argument and then "pop" it.
  msg.assign( ring[_front].bytes.begin( ), ring[_front].bytes.end( ) );
  timeStamp = ring[_front].timeStamp;

  // Update front
  front = ( front+1 )%ringSize;
  return true;
}
#undef RTMIDI_CLASSNAME

//*********************************************************************//
// Common InternalMidiApi Definitions
//*********************************************************************//

#define RTMIDI_CLASSNAME "InternalMidiApi"
#if 0
InternalMidiApi :: InternalMidiApi( )
  : MidiApi( ), ignoreFlags( 7 ), doInput( false ), firstMessage( true ),
    userCallback( 0 ),
    continueSysex( false )
{
}
#endif

void InternalMidiApi :: setCallback( MidiInterface * callback )
{
  if ( userCallback ) {
    error( RTMIDI_ERROR( gettext_noopt( "A callback function is already set." ),
                         Error::WARNING ) );
    return;
  }

  if ( !callback ) {
    error( RTMIDI_ERROR( gettext_noopt( "The callback function value is invalid." ),
                         Error::WARNING ) );
    return;
  }

  userCallback = callback;
}

void InternalMidiApi :: cancelCallback( )
{
  if ( !userCallback ) {
    error( RTMIDI_ERROR( gettext_noopt( "No callback function was set." ),
                         Error::WARNING ) );
    return;
  }

  userCallback->deleteMe( );
  userCallback = 0;
}

#if 0
double InternalMidiApi :: getMessage( std::vector<unsigned char>& message )
{
  message.clear( );

  MidiQueueInterface * queueCallback = dynamic_cast<MidiQueueInterface *>(userCallback);

  if ( !queueCallback ) {
    error( RTMIDI_ERROR( gettext_noopt( "Returning an empty MIDI message as all input is handled by a callback function." ),
                         Error::WARNING ) );
    return 0.0;
  }

  return queueCallback->getMessage(message);
}
#endif

void InternalMidiApi :: ignoreTypes( bool midiSysex, bool midiTime, bool midiSense )
{
  ignoreFlags = 0;
  if ( midiSysex ) ignoreFlags = IGNORE_SYSEX;
  if ( midiTime ) ignoreFlags |= IGNORE_TIME;
  if ( midiSense ) ignoreFlags |= IGNORE_SENSING;
}

#undef RTMIDI_CLASSNAME


RTMIDI_NAMESPACE_END
