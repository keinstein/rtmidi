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

#ifndef RTMIDI_JACK_H
#define RTMIDI_JACK_H

//*********************************************************************//
// API: UNIX JACK
//
// Written primarily by Alexander Svetalkin, with updates for delta
// time by Gary Scavone, April 2011.
//
// *********************************************************************//

#if defined( __UNIX_JACK__ )

#include "RtMidi.h"
#include "src/RtMidi-internal.h"

// JACK header files
#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/ringbuffer.h>
#ifdef HAVE_SEMAPHORE
#include <semaphore.h>
#endif

#include <sstream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <functional>
#include <cerrno>


RTMIDI_NAMESPACE_START

#define JACK_RINGBUFFER_SIZE 16384 // Default size for ringbuffer


struct JackMidi;
class MidiJack;


#define RTMIDI_CLASSNAME "JackSequencer"
template <int locking=1>
class JackSequencer {
public:
  JackSequencer( const std::string& n )
    : mutex(), client( nullptr ), ports( )
  {
    init ( n );
#if 0
    if ( locking ) {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init( &attr );
      pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_NORMAL );
      if ( pthread_mutex_init( &mutex, &attr ) ) {
        throw RTMIDI_ERROR( gettext_noopt( "Could not initialise the JACK client mutex" ),
                            Error::MEMORY_ERROR );
      }
    }
#endif
  }

  ~JackSequencer( )
  {
    {
      std::lock_guard<std::mutex> lock ( mutex );
      if ( client ) {
        jack_deactivate ( client );
        // the latter doesn't flush the queue
        jack_client_close ( client );
        client = 0;
      }
    }
#if 0
    if ( locking ) {
      pthread_mutex_destroy( &mutex );
    }
#endif
  }

  void init() {
    if ( !client ) {
      init( "" );
    }
  }

  bool setName( const std::string& ) {
    /* we don't can't rename the client */
    return false;
  }

  std::string getName( ) {
    return jack_get_client_name( client );
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
#if 0
    scoped_lock<locking> lock ( mutex );
#else
    std::lock_guard<std::mutex> lock ( mutex );
#endif
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
#if 0
    scoped_lock<locking> lock ( mutex );
#else
    std::lock_guard<std::mutex> lock ( mutex );
#endif
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

  size_t getBufferSize() {
    return jack_get_buffer_size( client );
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

  void addApiPort(JackMidi * port) {
    ports.add(port);
  }

  void removeApiPort(JackMidi * port) {
    ports.remove(port);
  }
protected:
  std::mutex mutex;
  jack_client_t * client;
  typedef SequencerPortList<JackMidi> ApiPortList;
  ApiPortList ports;


  void init( const std::string & name )
  {
#if 0
    scoped_lock<locking> lock ( mutex );
#else
    std::lock_guard<std::mutex> lock ( mutex );
#endif
    if ( client ) return;
    if ( ( client = jack_client_open( name.c_str( ),
                                 JackNoStartServer,
                                 NULL ) ) == 0 ) {
      client = NULL;
      throw RTMIDI_ERROR( gettext_noopt( "Could not connect to JACK server. Is it runnig?" ),
                          Error::NO_DEVICES_FOUND );
      return;
    }

    jack_set_process_callback( client, Callback, this );
    // don't activate the client as we might want to set further callbacks
    jack_activate( client );
  }

  static int Callback ( jack_nframes_t nframes, void * arg );
};
typedef JackSequencer<1> LockingJackSequencer;
//typedef JackSequencer<0> NonLockingJackSequencer;
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "JackPortDescriptor"
struct JackPortDescriptor : public PortDescriptor
{
  JackPortDescriptor( std::shared_ptr<LockingJackSequencer>  &s = JackPortDescriptor::main_seq )
    : seq( s ), port( nullptr )
  {
  }
  JackPortDescriptor( const char * portname,
                      std::shared_ptr<LockingJackSequencer> &s = JackPortDescriptor::main_seq )
    : seq( s )
  {
    port = seq->getPort( portname );
  }

  JackPortDescriptor( JackPortDescriptor& other,
                      std::shared_ptr<LockingJackSequencer> & s = JackPortDescriptor::main_seq )
    : seq( s )
  {
#if 0
    port = other.port;
#else
    // assign port to the permanent sequencer
    port = other.clonePort( seq );
#endif
  }

  ~JackPortDescriptor( )
  {
  }

  MidiApi * getApi ( unsigned int capabilities ) const;

  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq->getPortName( port, flags );
  }

  const std::string getClientName( ) {
    return seq->getName();
  }
  int getCapabilities( ) const {
    return seq->getPortCapabilities( port );
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
  jack_port_t * clonePort( std::shared_ptr<JackSequencer<i> > &s ) const {
    const char * portName = jack_port_name( port );
    return s->getPort( portName );
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

  static void allocateSequencer( const std::string & clientName) {
    if (!main_seq) {
      main_seq = std::make_shared<LockingJackSequencer>( clientName );
    }
  }

  static PortList getPortList( int capabilities,
                               std::shared_ptr<LockingJackSequencer> & s );

  //operator jack_port_t * ( ) const { return port; }

protected:
  static std::shared_ptr<LockingJackSequencer>  main_seq;
  std::shared_ptr<LockingJackSequencer> seq;

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

  explicit JackPortDescriptor( jack_port_t * other,
                               std::shared_ptr<LockingJackSequencer> & s)
    : seq(s)
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
  }
};

#undef RTMIDI_CLASSNAME

/*! A structure to hold variables related to the JACK API
  implementation.

  \note After all sequencer handling is covered by the \ref
  JackSequencer class, we should make seq to be a pointer in order
  to allow a common client implementation.
*/

#define RTMIDI_CLASSNAME "JackMidi"
class JackMidi : public JackPortDescriptor {
protected:
  friend class MidiJack;
  friend class JackSequencer<true>;
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
#if 0
  /*! Sequencer object
    : This must be deleted _before_ the MIDI data to avoid
    segmentation faults while queued data is still in the ring buffer. */
  NonLockingJackSequencer * seq;
#endif

  /*
    JackMidi( )
    : seq( )
    {
    init( );
    }
  */
  JackMidi( MidiJack * inputData_,
            const std::string & clientName,
            std::shared_ptr<LockingJackSequencer> & s = JackPortDescriptor::main_seq )
    : JackPortDescriptor( s ),
      stateflags( RUNNING ),
      state_response( OPEN ),
      local( 0 ),
      buffSize( 0 ),
      buffMessage( 0 ),
      lastTime( 0 ),
      api( inputData_  /* ,
                           seq( new NonLockingJackSequencer( clientName, *this ) */ )
  {
    if (!seq) {
      JackPortDescriptor::allocateSequencer( clientName );
      seq = s;
    }
    if (s) {
      jack_nframes_t size = s->getBufferSize();
      buffSize    = jack_ringbuffer_create( size );
      buffMessage = jack_ringbuffer_create( size );
    }
#ifdef HAVE_SEMAPHORE
    sem_init( &sem_cleanup, 0, 0 );
    sem_init( &sem_needpost, 0, 0 );
#endif
    seq->addApiPort(this);
  }

#if 0
  /**
   * Create output midi data.
   *
   * \param clientName
   *
   * \return
   */
  JackMidi( std::shared_ptr<LockingJackSequencer> & s = JackPortDescriptor::main_seq )
    : JackPortDescriptor( s ),
      stateflags( RUNNING ),
      state_response( OPEN ),
      local( 0 ),
      lastTime( 0 ),
      api( ) /* ,
                seq( new NonLockingJackSequencer( clientName, *this )*/
  {
    if (s) {
      jack_nframes_t size = s->getBufferSize();
      buffSize    = jack_ringbuffer_create( size );
      buffMessage = jack_ringbuffer_create( size );
    }
#ifdef HAVE_SEMAPHORE
    sem_init( &sem_cleanup, 0, 0 );
    sem_init( &sem_needpost, 0, 0 );
#endif
    seq->addApiPort(this);
  }
#endif

  ~JackMidi( )
  {
    seq->removeApiPort(this);
    if ( local )
      try {
        deletePort( );
      } catch ( const Error& e ) {
        e.printMessage( std::cerr );
      }
#if 0
    if ( seq )
      delete seq;
#endif
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

  void init( ) {
#if 0
    seq->init( );
#endif
  }

  size_t maxSysExSize() const {
    return std::min(seq->getBufferSize(),
                    jack_ringbuffer_write_space(buffMessage));
  }

  void setClientName( const std::string & clientName ) {
    seq->setName(clientName);
  }

  void setRemote( const JackPortDescriptor& o ) {
    port = o.clonePort( seq );
  }

  void connectFrom( const JackPortDescriptor& from ) {
    setRemote( from );
    seq->connectPorts( port, local );
  }

  void connectTo( const JackPortDescriptor& to ) {
    setRemote( to );
    seq->connectPorts( local, port );
  }

#if 0
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
#endif

#if 0
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
#endif

  using JackPortDescriptor::getPortList;
  PortList getPortList( int capabilities ) {
    return JackPortDescriptor::getPortList( capabilities, seq );
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
        return Pointer<PortDescriptor>( new JackPortDescriptor( local, seq ) );
      }
    } else {
      return Pointer<PortDescriptor>( new JackPortDescriptor( *this, seq ) );
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

#define RTMIDI_CLASSNAME "MidiJack"
class MidiJack : public InternalMidiApi
{
public:
  typedef InternalMidiApi base;

  MidiJack( const std::string& clientName );
  ~MidiJack( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::UNIX_JACK; };
  unsigned int getCapabilities ( ) throw ( ) {
    return PortDescriptor::INOUTPUT
      | PortDescriptor::VIRTUAL
      | PortDescriptor::UNLIMITED;
  }
  bool hasVirtualPorts( ) const { return true; }
  size_t maxSysExSize() const;
  void openVirtualPort( const std::string& portName,
                        unsigned int capabilities );
  using base::openPort;
  void openPort( const PortDescriptor& port,
                 const std::string& portName,
                 unsigned int capabilities );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& clientName );
  void setPortName( const std::string& portName );
#if 0
  unsigned int getPortCount( void );
  std::string getPortName( unsigned int portNumber );
  void openPort( unsigned int portNumber,
                 const std::string& portName,
                 unsigned int capabilities );
#endif
  void sendMessage( const unsigned char * message, size_t size );

public:
  JackMidi * midi;
  void initialize( const std::string& clientName );
};
#undef RTMIDI_CLASSNAME

RTMIDI_NAMESPACE_END
#endif // __UNIX_JACK__
#endif // RTMIDI_JACK_H
