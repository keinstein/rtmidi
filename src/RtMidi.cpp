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
#include "src/RtMidi-internal.h"
#if defined( __UNIX_JACK__ )
#include "src/JACK.h"
#endif
#if defined( __LINUX_ALSA__ )
#include "src/ALSA.h"
#endif
#include <sstream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <functional>
#include <cerrno>


// Default for Windows is to add an identifier to the port names; this
// flag can be defined (e.g. in your project file) to disable this behaviour.
//#define RTMIDI_DO_NOT_ENSURE_UNIQUE_PORTNAMES

// **************************************************************** //
//
// Common helpers.
//
// **************************************************************** //



extern "C" {

  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const enum rtmidi::ApiType rtmidi_compiled_system_apis[] =
    {
#if defined( __MACOSX_CORE__ )
     rtmidi::MACOSX_CORE,
#endif
#if defined( __LINUX_ALSA__ )
     rtmidi::LINUX_ALSA,
#endif
#if defined( __WINDOWS_MM__ )
     rtmidi::WINDOWS_MM,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_system_apis =
    sizeof( rtmidi_compiled_system_apis )/sizeof( rtmidi_compiled_system_apis[0] );

  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const enum rtmidi::ApiType rtmidi_compiled_software_apis[] =
    {
#if defined( __UNIX_JACK__ )
     rtmidi::UNIX_JACK,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_software_apis =
    sizeof( rtmidi_compiled_software_apis )/sizeof( rtmidi_compiled_software_apis[0] );


  // The order here will control the order of RtMidi's API search in
  // the constructor.
  extern "C" const enum rtmidi::ApiType rtmidi_compiled_other_apis[] =
    {
     rtmidi::UNSPECIFIED,
     rtmidi::ALL_API,
#if defined( __RTMIDI_DUMMY__ )
     rtmidi::DUMMY,
#endif
    };
  extern "C" const size_t rtmidi_num_compiled_other_apis =
    sizeof( rtmidi_compiled_other_apis )/sizeof( rtmidi_compiled_other_apis[0] );



  extern "C" const struct rtmidi::api_name rtmidi_api_names[rtmidi::NUM_APIS] =
    {
     [rtmidi::UNSPECIFIED]
     = { rtmidi::UNSPECIFIED, "unspecified" ,
         N_( "Automatic backend selection" ) },
     [rtmidi::MACOSX_CORE]
     = { rtmidi::MACOSX_CORE, "core" ,        N_( "Core MIDI" ) },
     [rtmidi::LINUX_ALSA]
     = { rtmidi::LINUX_ALSA,  "alsa" ,        N_( "ALSA" ) },
     [rtmidi::UNIX_JACK]
     = { rtmidi::UNIX_JACK,   "jack" ,        N_( "JACK" ) },
     [rtmidi::WINDOWS_MM]
     = { rtmidi::WINDOWS_MM,  "winmm" ,
         N_( "Windows Multimedia" ) },
     [rtmidi::DUMMY]
     = { rtmidi::DUMMY,       "dummy" ,
         N_( "Dummy/NULL device" ) },
     [rtmidi::WINDOWS_KS]
     = { rtmidi::WINDOWS_KS,  "winks" ,
         N_( "DirectX/Kernel Streaming" ) },
     [rtmidi::ALL_API]
     = { rtmidi::ALL_API,     "allapi" ,
         N_( "All available MIDI systems" ) },
    };
  extern "C" const unsigned int rtmidi_num_api_names = rtmidi::NUM_APIS;
  // = sizeof( rtmidi_api_names )/sizeof( rtmidi_api_names[0] );
}


RTMIDI_NAMESPACE_START

ignore_type ignore_flags;


// Define API names and display names.
// Must be in same order as API enum.

#if 0
#if !defined( __LINUX_ALSA__ ) && !defined( __UNIX_JACK__ ) && !defined( __MACOSX_COREMIDI__ ) && defined( __WINDOWS_MM__ )
struct pthread_mutex_t;
constexpr inline int pthread_mutex_lock( pthread_mutex_t* ) { return 0; }
constexpr inline int pthread_mutex_unlock( pthread_mutex_t* ) { return 0; }
#endif
#endif

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
  std::va_list args, copyargs;
  va_start( args, line_number );
  va_copy( copyargs, args );
  size_t length;
  length = vsnprintf( NULL, 0, message, args );
  if ( length > 0 ) {
    message_.resize( length+1 );
    std::vsnprintf( &( message_[0] ), length+1, message, copyargs );
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
  va_end( copyargs );

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




void Midi :: error( const Error & e )
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
    for ( auto api : queryApis ) {
      try {
        MidiApi * rtapi_ = openMidiApi( api );
        if ( rtapi_ ) {
          apis.push_back( MidiApiPtr( rtapi_ ) );
        }
      } catch ( const Error& e ) {
        if ( e.getType( ) != Error::NO_DEVICES_FOUND )
          throw;
      }
    }
    if (api == rtmidi::ALL_API)
      return;
  } else {
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
    return;
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
       apis.push_back(rtapi_);
       break;
    }
  }
  // Fallback: keep one api
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


void MidiApi :: error( const Error & e )
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

  if ( _size == 0 ) {
    return false;
  }

  // Copy queued message to the vector pointer argument and then "pop" it.
  msg.assign( ring[_front].bytes.begin( ), ring[_front].bytes.end( ) );
  timeStamp = ring[_front].timeStamp;

  // Update front
  front = ( front+1 )%ringSize;
  return true;
}
#undef RTMIDI_CLASSNAME


void MidiInterface::sysexChunk ( double timestamp,
                                 unsigned char * begin,
                                 ptrdiff_t size,
                                 int connectionId ) throw()
{
  rtmidiUnused(timestamp);
  rtmidiUnused(begin);
  rtmidiUnused(size);
  rtmidiUnused(connectionId);
}

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

void InternalMidiApi :: setCallback( SplitSysexMidiInterface * callback )
{
  if ( !callback ) {
    error( RTMIDI_ERROR( gettext_noopt( "The callback function value is invalid." ),
                         Error::WARNING ) );
    return;
  }

  // TODO: For compatibility with older RtMidi versions we should issue a warning, here.
  // This is not possible as we don't have access to the standard callback.
  if ( userCallback ) {
  #if 0
    error( RTMIDI_ERROR( gettext_noopt( "A callback function is already set." ),
                         Error::WARNING ) );
  #endif
    cancelCallback();
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
