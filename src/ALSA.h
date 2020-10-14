/**********************************************************************/
/*! \s Midi
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

//*********************************************************************//
// API: LINUX ALSA SEQUENCER
//*********************************************************************//

// API information found at:
// - http://www.alsa-project.org/documentation.php#Library

// The ALSA Sequencer API is based on the use of a callback function for
// MIDI input.
//
// Thanks to Pedro Lopez-Cabanillas for help with the ALSA sequencer
// time stamps and other assorted fixes!!!

// If you don't need timestamping for incoming MIDI events, define the
// preprocessor definition AVOID_TIMESTAMPING to save resources
// associated with the ALSA sequencer queues.

#ifndef RTMIDI_ALSA_H
#define RTMIDI_ALSA_H

#if defined( __LINUX_ALSA__ )
#include "RtMidi.h"
#include "src/RtMidi-internal.h"
#include <pthread.h>
#include <sys/time.h>

// ALSA header file.
#include <alsa/asoundlib.h>
#include <thread>
#include <condition_variable>
//#include <sstream>
//#include <cstring>
//#include <cctype>
//#include <algorithm>
//#include <functional>
//#include <cerrno>

RTMIDI_NAMESPACE_START


/*! An abstraction layer for the ALSA sequencer layer. It provides
  the following functionality:
  - dynamic allocation of the sequencer
  - optionallay avoid concurrent access to the ALSA sequencer,
  which is not thread proof. This feature is controlled by
  the parameter \ref locking.
*/

// helper functions to avoid compiler warnings
template<class T>
inline bool is_negative( const T& x ) {
  return x < 0;
}

template<>
inline bool is_negative<unsigned int>( const unsigned int& ) {
  return false;
}
template<>
inline bool is_negative<unsigned long>( const unsigned long& ) {
  return false;
}


class AlsaMidi;
class MidiAlsa;

#define RTMIDI_CLASSNAME "AlsaSequencer"

void printalsaevent(const snd_seq_event_t * ev);

template <int locking=1>
class AlsaSequencer {
public:
  typedef AlsaSequencer<1> LockingAlsaSequencer;
  class InputQueue {
  public:
    InputQueue():queue_id(-1) {
    }
    ~InputQueue() {}

    void preparePort( snd_seq_port_info_t * pinfo ) {
#ifndef AVOID_TIMESTAMPING
      snd_seq_port_info_set_timestamping( pinfo, 1 );
      snd_seq_port_info_set_timestamp_real( pinfo, 1 );
      snd_seq_port_info_set_timestamp_queue( pinfo, queue_id );
#endif
    }

    void start(std::shared_ptr<LockingAlsaSequencer> & seq) {
      start(seq->seq);
    }

    void start(snd_seq_t * seq) {
      if (queue_id >= 0) return;
#ifndef AVOID_TIMESTAMPING
      queue_id = snd_seq_alloc_queue( seq/*, "Midi Queue" */);
      // Set arbitrary tempo ( mm=100 ) and resolution ( 240 )
      snd_seq_queue_tempo_t * qtempo;
      snd_seq_queue_tempo_alloca( &qtempo );
      snd_seq_queue_tempo_set_tempo( qtempo, 600000 );
      snd_seq_queue_tempo_set_ppq( qtempo, 240 );
      snd_seq_set_queue_tempo( seq, queue_id, qtempo );
      snd_seq_start_queue( seq, queue_id, NULL );
      snd_seq_drain_output( seq );
#endif
    }

    void stop( std::shared_ptr<LockingAlsaSequencer> & seq ) {
      stop(seq->seq);
    }
    void stop( snd_seq_t * seq ) {
#ifndef AVOID_TIMESTAMPING
      snd_seq_free_queue( seq, queue_id );
      queue_id = -1;
#endif
    }



  protected:
    int queue_id; // an input queue is needed to get timestamped events
  };

  class ThreadFunction {
  public:
    ThreadFunction(AlsaSequencer & s): seq(s) {}
    ThreadFunction(ThreadFunction && o):seq(o.seq) {}

    void operator () () { seq.inputThread(); }

  protected:
    AlsaSequencer & seq;
  };

#if 0
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
#endif

  AlsaSequencer( const std::string& name )
    : mutex(),
      seq( nullptr ),
      trigger_fds{-1,-1}
  {
    init( seq );
    // Save our api-specific connection information.

    if ( pipe( trigger_fds ) == -1 ) {
      snd_seq_close( seq );
      throw RTMIDI_ERROR( gettext_noopt( "Error creating ALSA MIDI pipe objects." ),
                          Error::DRIVER_ERROR );
      return;
    }

#if 0
    // create output Coder
    int result = snd_midi_event_new( 0, &in_coder );
    if ( result < 0 ) {
      snd_seq_close( seq );
      close ( trigger_fds[0] );
      close ( trigger_fds[1] );

      throw RTMIDI_ERROR( gettext_noopt( "Error initializing ALSA MIDI event parser." ),
                          Error::DRIVER_ERROR );
      return;
    }
    snd_midi_event_init( in_coder );
#endif

    try {
      {
#if 0
        scoped_lock<locking> lock ( mutex );
#else
        std::lock_guard<std::mutex> lock ( mutex );
#endif
        snd_seq_set_client_name( seq, name.c_str( ) );
#if 0
        queue.start( seq );
#endif
      }
      startThread();
    } catch (...) {
      snd_seq_close( seq );
      close ( trigger_fds[0] );
      close ( trigger_fds[1] );
      throw;
    }
  }

  ~AlsaSequencer( )
  {
    if ( seq ) {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
#if 0
      queue.stop( seq );
#endif
      // Shutdown the input thread.
      {
        threadCommand = THREAD_STOP;
        int res, data = -1;
        res = write( trigger_fds[1], &data, sizeof( int ) );
        ( void ) res;
      }
      loop.join();
#if 0
      if ( in_coder ) {
        snd_midi_event_free( in_coder );
        in_coder = nullptr;
      }
#endif


      snd_seq_close( seq );
      close ( trigger_fds[0] );
      close ( trigger_fds[1] );
      seq = 0;
    }
#if 0
    if ( locking ) {
      pthread_mutex_destroy( &mutex );
    }
#endif
  }

  int setName( const std::string& name ) {
    /* we don't want to rename the client after opening it. */
    if ( seq ) {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
      return snd_seq_set_client_name( seq, name.c_str( ) );
    }
    return 0;
  }

  std::string getName( ) const {
    snd_seq_client_info_t  * cinfo;
    snd_seq_client_info_alloca( &cinfo );
    {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( const_cast<std::mutex &>( mutex ) );
#endif
      snd_seq_get_client_info( seq, cinfo );
    }
    std::string retval = snd_seq_client_info_get_name( cinfo );
    return retval;
  }


  std::string getPortName( int client, int port, int flags ) const {
    snd_seq_client_info_t * cinfo;
    snd_seq_client_info_alloca( &cinfo );
    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );
    {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( const_cast<std::mutex & >( mutex ) );
#endif
      snd_seq_get_any_client_info( seq, client, cinfo );
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
    {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
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
  }

  int getPortCapabilities( int client, int port ) {
    init( );
    snd_seq_port_info_t * pinfo;
    snd_seq_port_info_alloca( &pinfo );
    {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
      snd_seq_get_any_port_info( seq, client, port, pinfo );
    }
    unsigned int caps = snd_seq_port_info_get_capability( pinfo );
    int retval = ( caps & ( SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ ) )?
      (int)PortDescriptor::INPUT : 0;
    if ( caps & ( SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE ) )
      retval |= PortDescriptor::OUTPUT;
    return retval;
  }

  int getNextClient( snd_seq_client_info_t * cinfo ) {
    init( );
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
    return snd_seq_query_next_client ( seq, cinfo );
  }

  int getNextPort( snd_seq_port_info_t * pinfo ) {
    init( );
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
    return snd_seq_query_next_port ( seq, pinfo );
  }

  int createPort ( snd_seq_port_info_t * pinfo, MidiAlsa * port ) {
    init( );
#if 0
    queue.preparePort( pinfo );
#endif

#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif

    int retval = snd_seq_create_port( seq, pinfo );
    if (!retval) {
      ports.add(snd_seq_port_info_get_port( pinfo ), port);
    }
    return retval;
  }

  void deletePort( int portid, MidiAlsa * port ) {
    init( );
#if 0
    scoped_lock<locking> lock ( mutex );
#else
    std::lock_guard<std::mutex> lock ( mutex );
#endif
    ports.remove(port);
    snd_seq_delete_port( seq, portid );
  }

  snd_seq_port_subscribe_t * connectPorts( const snd_seq_addr_t& from,
                                           const snd_seq_addr_t& to,
                                           bool real_time ) {
    init( );
    snd_seq_port_subscribe_t * subscription;

    if ( snd_seq_port_subscribe_malloc( &subscription ) < 0 ) {
      throw RTMIDI_ERROR( gettext_noopt( "Could not allocate ALSA port subscription." ),
                          Error::DRIVER_ERROR );
      return nullptr;
    }
    snd_seq_port_subscribe_set_sender( subscription, &from );
    snd_seq_port_subscribe_set_dest( subscription, &to );
    if ( real_time ) {
      snd_seq_port_subscribe_set_time_update( subscription, 1 );
      snd_seq_port_subscribe_set_time_real( subscription, 1 );
    }
    {
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
      if ( snd_seq_subscribe_port( seq, subscription ) ) {
        snd_seq_port_subscribe_free( subscription );
        subscription = nullptr;
        throw RTMIDI_ERROR( gettext_noopt( "Error making ALSA port connection." ),
                            Error::DRIVER_ERROR );
        return 0;
      }
    }
    return subscription;
  }

  void closePort( snd_seq_port_subscribe_t *  subscription ) {
    init( );
#if 0
    scoped_lock<locking> lock ( mutex );
#else
    std::lock_guard<std::mutex> lock ( mutex );
#endif
    snd_seq_unsubscribe_port( seq, subscription );
  }


  int maxSysExSize() {
    // Find a better way to do this:
    snd_seq_client_pool_t * pool;
    snd_seq_client_pool_alloca( &pool );
    snd_seq_get_client_pool( seq, pool );
    return (std::min(snd_seq_client_pool_get_input_pool( pool ),
                     snd_seq_client_pool_get_output_pool( pool )) / 4-1)
      *sizeof (snd_seq_event_t);
  }

#if 0
  // TODO: make this function working
  size_t getInputSpace( int port ) {
    snd_seq_query_subscribe_t * subscription;
    snd_seq_port_subscribe_alloca(&subscription);
    snd_seq_queue_info_t * queueinfo;
    snd_seq_queue_info_alloca(&queueinfo);
    snd_seq_client_info_t * clientinfo;
    snd_seq_client_info_alloca(&clientinfo);

    snd_seq_addr_t subscriber;
    int err = 0;
    size_t i = 0;
    if (!subscription) return 0;
    while (err >= 0) {
      snd_seq_query_subscribe_set_index( subscription, i );
      snd_seq_query_subscribe_set_type( subscription, SND_SEQ_QUERY_SUBS_READ );
      err = snd_seq_query_port_subscribers( seq, subscription );
      int client = snd_seq_query_subscribe_get_client( subscription );
      int port   = snd_seq_query_subscribe_get_port( subscription );
      int queue  = snd_seq_query_subscribe_get_queue( subscription );
      err = snd_seq_get_any_client_info( seq, client, clientinfo );
      int lostEvents             = snd_seq_client_info_get_event_lost( clientinfo );
      snd_seq_client_type_t type = snd_seq_client_info_get_type ( clientinfo );
      int clientid;
      switch ( type ) {
      case SND_SEQ_KERNEL_CLIENT:
        clientid = snd_seq_client_info_get_card ( clientinfo ); // hardware
        break;
      case SND_SEQ_USER_CLIENT:
        clientid = snd_seq_client_info_get_pid ( clientinfo );  // software
        break;
      default:
        clientid = -1;
      }

      err = snd_seq_get_queue_info ( clientseq, queue, queueinfo );
      ++i;
    }
  }
#endif

  /**
   * Send a MIDI message to the ALSA Sequencer.
   *
   * This function should be thread safe.
   *
   * \param ev Event to be sent.
   */
  void sendMessage(const snd_seq_event_t & ev) {
    // Send the event.
    static const std::chrono::microseconds sleeptime( 10 );
    int result = 0 ;
    if ((result = snd_seq_event_length( const_cast<snd_seq_event_t*>( &ev ))) < 0 ) {
        error( RTMIDI_ERROR1( gettext_noopt( "Error sending ALSA MIDI message with negative length %d." ),
                              Error::WARNING,
                              result ) );
        return;
    }
    printalsaevent(&ev);
    while (1) {
      // TODO: find a way to cirumvent stupid ALSA behaviour:
      //       in case an event cannot be delivered to a software device
      //       ALSA clears the complete queue instead of
      //       just throwing away the new message
      {
#if 0
        scoped_lock<locking> lock ( mutex );
#else
        std::lock_guard<std::mutex> lock ( mutex );
#endif
        result = snd_seq_event_output_direct( seq, const_cast<snd_seq_event_t*>(&ev) );
      }
      if (result != -EAGAIN && result != -EWOULDBLOCK)
        break;
      {
#if 0
        scoped_lock<locking> lock ( mutex );
#else
        std::lock_guard<std::mutex> lock ( mutex );
#endif
        snd_seq_sync_output_queue( seq );
      }
      //std::this_thread::sleep_for(sleeptime);
    }
    if ( result < 0 ) {
      error( RTMIDI_ERROR2( gettext_noopt( "Error sending ALSA MIDI message to port.\nError %d: %s" ),
                            Error::WARNING,
                            -result,
                            snd_strerror(-result) ) );
    }
    // snd_seq_drain_output( seq ); // needed when output is not direct
  }

  bool startThread( /* void * userdata */ ) {
    try {
      threadStatus = THREAD_START;
      threadCommand = THREAD_RUN;
      loop = std::thread(ThreadFunction(*this));
      {
        std::unique_lock<std::mutex> lock(syncMutex);
        syncThread.wait(lock,[this](){
                               return (threadStatus == THREAD_STARTED) ||
                                 (threadStatus == THREAD_ERROR );
                             } );
      }
#if 0
      // Start the input queue
      // Start our MIDI input thread.
      pthread_attr_t attr;
      pthread_attr_init( &attr );
      pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
      pthread_attr_setschedpolicy( &attr, SCHED_OTHER );

      int err = pthread_create( &thread, &attr, MidiAlsa::alsaMidiHandler, userdata );
      pthread_attr_destroy( &attr );
      if ( err ) {
      }
#endif
    } catch(...) {
      throw RTMIDI_ERROR( gettext_noopt( "Error starting ALSA MIDI input thread!" ),
                          Error::THREAD_ERROR );
      return false;
    }
    return true;
  }

  void inputThread() throw() ;

  std::ostream & printPoolSizes( std::ostream & o ) {
    snd_seq_client_pool_t * pool;
    snd_seq_client_pool_alloca( &pool );
    snd_seq_get_client_pool( seq, pool );
    o << "Cell size" << sizeof (snd_seq_event_t) << std::endl;
    o << "Client: " << snd_seq_client_pool_get_client( pool ) << std::endl;
    o << "Available input size: " << snd_seq_client_pool_get_input_free( pool ) << std::endl;
    o << "Input pool size: " << snd_seq_client_pool_get_input_pool( pool ) << std::endl;
    o << "Available output size: " << snd_seq_client_pool_get_output_free( pool ) << std::endl;
    o << "Output pool size: " << snd_seq_client_pool_get_output_pool( pool ) << std::endl;
    o << "Output room size: " << snd_seq_client_pool_get_output_room( pool ) << std::endl;
    return o;
  }

protected:
  enum ThreadStatus
    {
     THREAD_START,
     THREAD_RUN,
     THREAD_STARTED,
     THREAD_STOP,
     THREAD_ERROR
    };

  typedef CopyOnWrite<std::vector<MidiAlsa *>> portlist_t;
  portlist_t ports;
#if 0
  pthread_mutex_t mutex;
#else
  std::mutex mutex;
#endif
  snd_seq_t * seq;
  std::thread loop;
  //  pthread_t dummy_thread_id;
  int trigger_fds[2];
  std::atomic<ThreadStatus> threadCommand,threadStatus;
  std::mutex syncMutex;
  std::condition_variable syncThread;



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

  template<class E>
  void error(const E & e) throw();

  template<class E>
  void error(const E & e, std::unique_lock<portlist_t> & lock) throw();

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
#if 0
      scoped_lock<locking> lock ( mutex );
#else
      std::lock_guard<std::mutex> lock ( mutex );
#endif
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
    }
  }
};
#undef RTMIDI_CLASSNAME
typedef AlsaSequencer<1> LockingAlsaSequencer;
//typedef AlsaSequencer<0> NonLockingAlsaSequencer;

#define RTMIDI_CLASSNAME "AlsaPortDescriptor"
struct AlsaPortDescriptor
  : public PortDescriptor,
    public snd_seq_addr_t
{
  AlsaPortDescriptor( std::shared_ptr<LockingAlsaSequencer>  &s = AlsaPortDescriptor::main_seq )
    : snd_seq_addr_t{0,0},
      seq( s )
  {
  }

  explicit AlsaPortDescriptor( snd_seq_addr_t& other,
                               std::shared_ptr<LockingAlsaSequencer>  &s = AlsaPortDescriptor::main_seq )
    : snd_seq_addr_t( other ),
      seq( s ) {
  }

  ~AlsaPortDescriptor( ) {}
  InternalMidiApi * getApi( unsigned int capabilities ) const;

  std::string getName( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) {
    return seq->getPortName( client, port, flags );
  }

  std::string getClientName( ) {
    return seq->getName();
  }

  int getCapabilities( ) const {
    if ( !client ) return 0;
    return seq->getPortCapabilities( client, port );
  }

  virtual bool operator == ( const PortDescriptor& o ) {
    const AlsaPortDescriptor * desc = dynamic_cast<const AlsaPortDescriptor*>( &o );
    if ( !desc ) return false;
    return client == desc->client && port == desc->port;
  }

  static void allocateSequencer( const std::string & clientName) {
    if (!main_seq) {
      main_seq = std::make_shared<LockingAlsaSequencer>( clientName );
    }
  }

  static PortList getPortList( int capabilities,
                               std::shared_ptr<LockingAlsaSequencer> & s );

protected:
  static std::shared_ptr<LockingAlsaSequencer>  main_seq;
  std::shared_ptr<LockingAlsaSequencer> seq;

  explicit AlsaPortDescriptor( int c,
                               int p,
                               std::shared_ptr<LockingAlsaSequencer>  &s = AlsaPortDescriptor::main_seq )
    : seq( s )
  {
    client = c;
    port = p;
  }

};
#undef RTMIDI_CLASSNAME


/*! A structure to hold variables related to the ALSA API
  implementation.

  \note After all sequencer handling is covered by the \ref
  AlsaSequencer class, we should make seq to be a pointer in order
  to allow a common client implementation.
*/
#define RTMIDI_CLASSNAME "AlsaMidi"

constexpr const size_t minChunkSize = 32;
constexpr size_t calculateAlsaBufferSize() {
  // the ALSA sequencer buffer is organized in cells that have the same size as
  // snd_seq_event_t. In order to be economic with the buffer space SysEx messages
  // should be split into chunks that have a length that is a multiple of the size of the
  // buffer cells.
  // So we round 32 up to the next cell border.
  return  minChunkSize - (minChunkSize % sizeof(snd_seq_event_t)) + sizeof(snd_seq_event_t);
}

class AlsaMidi: public AlsaPortDescriptor {
public:
  /*
    AlsaMidi( )
    : seq( )
    {
    init( );
    }
  */
  AlsaMidi( const std::string & clientName,
            std::shared_ptr<LockingAlsaSequencer> & s = AlsaPortDescriptor::main_seq )
    : AlsaPortDescriptor( s ),
      currentMessage(),
      lastTime({0,0}),
      currentSysexSize(0),
      local({0,0}),
      //port(-1),
      subscription_in( nullptr ),
      subscription_out( nullptr ),
      in_coder( nullptr ),
      out_coder( nullptr ),
      queue(),
      buffer(calculateAlsaBufferSize()),
      rcv_sysex_size(0),
      rcv_sysex_count(0)
  {
    if (!seq) {
      allocateSequencer( clientName );
      seq = s;
    }
  }

  ~AlsaMidi( )
  {
    try {
      if ( local.client && local.port )
        deletePort( );
    } catch ( const Error& e ) {
      // we don't have access to the error handler
      e.printMessage( );
    }
    if ( in_coder ) {
      snd_midi_event_free( in_coder );
      in_coder = nullptr;
    }
    if ( out_coder ) {
      snd_midi_event_free( out_coder );
      out_coder = nullptr;
    }
  }

  void init ( ) {
    local.port = 0;
    local.client = 0;
    port = -1;
    subscription_in = nullptr;
    subscription_out = nullptr;
    in_coder = nullptr ;
    out_coder =  nullptr ;
  }


  void setRemote( const AlsaPortDescriptor * remote ) {
    port = remote->port;
    client = remote->client;
  }

  void connectPorts( const snd_seq_addr_t& remote,
                     const snd_seq_addr_t& local,
                     unsigned int capabilities ) {
    if ( capabilities & PortDescriptor::INPUT ) {
      if (subscription_in)
        throw  RTMIDI_ERROR( gettext_noopt( "ALSA input port has algready been connected." ),
                             Error::DRIVER_ERROR ) ;

      subscription_in = seq->connectPorts( remote, local, false );
    }
    if ( capabilities & PortDescriptor::OUTPUT ) {
      if (subscription_out)
        throw  RTMIDI_ERROR( gettext_noopt( "ALSA output port has algready been connected." ),
                             Error::DRIVER_ERROR ) ;
      subscription_out = seq->connectPorts( local, remote, true );
    }
  }


  void createCoder ( snd_midi_event_t ** coder ) {
    if ( !coder )  {

      throw RTMIDI_ERROR( gettext_noopt( "Tying to initialize null pointer as ALSA MIDI event parser." ),
                          Error::DRIVER_ERROR );
      return;
    }
    if ( *coder ) return;
    int result = snd_midi_event_new( buffer.size( ), coder );
    if ( result < 0 ) {

      throw RTMIDI_ERROR( gettext_noopt( "Error initializing ALSA MIDI event parser." ),
                          Error::DRIVER_ERROR );
      return;
    }
    snd_midi_event_init( *coder );
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
                  const std::string& portName );

  void deletePort( );

  void closePort(snd_seq_port_subscribe_t * &subscription) {
    if (!subscription) return;
    seq->closePort( subscription );
    snd_seq_port_subscribe_free( subscription );
    subscription = 0;
  }

  void closePort( ) {
    closePort( subscription_in );
    closePort( subscription_out );
  }

  void setName( const std::string& name ) {
    seq->setPortName( name, local.port );
  }

  size_t maxSysExSize() const {
    return seq->maxSysExSize();
  }

  void setClientName( const std::string& name ) {
    seq->setName( name );
  }

  //  void receiveEvent( snd_seq_event_t * ev) throw();
protected:
  // the follownig members will be used directly after
  // splitting devices from ports in the API
  MidiMessage currentMessage; // currently used by MidiAlsa
  snd_seq_real_time_t lastTime; // currently used by MidiAlsa
  size_t currentSysexSize;    // currently used by MidiAlsa

  snd_seq_addr_t local; /*!< Our port and client id. If client = 0 ( default ) this means we didn't aquire a port so far. */
  //  unsigned int portNum;
  snd_seq_port_subscribe_t * subscription_in, * subscription_out;
  snd_midi_event_t *in_coder, *out_coder;
  LockingAlsaSequencer::InputQueue queue;
  std::vector<unsigned char> buffer;
  size_t rcv_sysex_size ;
  int rcv_sysex_count;

  size_t  alsa2Midi( const snd_seq_event_t * event, MidiMessage & message ) {

    size_t s = snd_seq_event_length(const_cast<snd_seq_event_t *>( event ));
    if ( message.bytes.size() != s ) message.bytes.resize(s);
    return alsa2Midi( event,
                      message.bytes.data(),
                      message.bytes.size() );
  }
  long alsa2Midi( const snd_seq_event_t * event,
                  unsigned char * buffer,
                  long size ) {
    if (!in_coder) return 0;
#if 0
    std::cerr << +event->source.client << ":" << +event->source.port << "->"
              << +event->dest.client << ":" << +event->dest.port << std::endl;
#endif
    return snd_midi_event_decode( in_coder,
                                  buffer,
                                  size,
                                  const_cast<snd_seq_event_t *>( event ) );
  }
};
#undef RTMIDI_CLASSNAME


//#define PORT_TYPE( pinfo, bits ) ( ( snd_seq_port_info_get_capability( pinfo ) & ( bits ) ) == ( bits ) )

//*********************************************************************//
// API: LINUX ALSA
// Class Definitions: MidiAlsa
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiAlsa"
class MidiAlsa: public AlsaMidi,
                public InternalMidiApi {
public:
  typedef AlsaMidi base;
  typedef InternalMidiApi api;
  MidiAlsa( const std::string& clientName );
  ~MidiAlsa( void );
  ApiType getCurrentApi( void ) throw( ) { return rtmidi::LINUX_ALSA; };
  unsigned int getCapabilities ( ) throw ( ) {
    return PortDescriptor::INOUTPUT
      | PortDescriptor::VIRTUAL
      | PortDescriptor::UNLIMITED;
  }
  bool hasVirtualPorts( ) const { return true; }
  size_t maxSysExSize() const;
  void openVirtualPort( const std::string& portName,
                        unsigned int capabilities );
  void openPort( const PortDescriptor& port,
                 const std::string& portName,
                 unsigned int capabilities  );
  Pointer<PortDescriptor> getDescriptor( bool isLocal=false );
  PortList getPortList( int capabilities );
  void closePort( void );
  void setClientName( const std::string& portName ) {
    AlsaMidi::setClientName( portName );
  }
  void setPortName( const std::string& portName ) {
    AlsaMidi::setName( portName );
  }
  void sendMessage( const unsigned char * message, size_t size );
public:
  //static void * alsaMidiHandler( void * ptr ) throw( );
  void initialize( const std::string & clientName );
  double makeTimeStamp ( const snd_seq_event_t * event ) throw();
#if 0
  void doCallback( const snd_seq_event_t * event,
                   unsigned char* data,
                   ptrdiff_t size ) throw();
#endif

#if 0
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
                  MidiMessage& message ) throw();
#endif

  bool doAlsaEvent( snd_seq_event_t * event ) throw();

  friend class AlsaMidi; // for registering the callback

  void receiveEvent( snd_seq_event_t * ev) throw();
};


inline __attribute__( ( always_inline ) )
double MidiAlsa :: makeTimeStamp ( const snd_seq_event_t * event ) throw()
{
  double timeStamp;
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
    timeStamp = 0.0;
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

    timeStamp = x.tv_sec + x.tv_nsec * 1e-9;
  }
  //lastTime = event->time.time;
  return timeStamp;
}


#if 0
inline __attribute__( ( always_inline ) )
void MidiAlsa :: doCallback( const snd_seq_event_t * event,
                             unsigned char * data,
                             ptrdiff_t size) throw() {
  double timeStamp;
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
    timeStamp = 0.0;
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
    double time = x.tv_sec + x.tv_nsec * 1.0e-9;


    timeStamp = time;
  }
  lastTime = event->time.time;
  doMidiCallback( timeStamp, data, size );
#if 0
  if ( userCallback )
    userCallback->midiIn( message.timeStamp, message.bytes, message.size );
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
#endif
}
#endif

#if 0
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
size_t MidiAlsa :: doSysEx( snd_seq_event_t * event,
                            size_t old_size,
                            MidiMessage& message ) throw()
{
#if 0
  message.bytes.resize( old_size + event->data.ext.len );
  // whlat if resize fails?
  long nBytes = alsa2Midi( event,
                           &message.bytes[old_size],
                           message.bytes.size( ) - old_size );
  if ( nBytes > 0 ) {
    ++rcv_sysex_count;
    old_size += nBytes;
    if ( message.bytes[old_size - 1] == 0xF7 ) {
      old_size = 0;
      doMidiCallback( makeTimeStamp( event ),
                      message.bytes.data(),
                      message.bytes.size() );
    }
  }
  return old_size;
#endif
}
#endif

inline __attribute__( ( always_inline ) )
bool MidiAlsa :: doAlsaEvent( snd_seq_event_t * event ) throw() {
  // we don't have lengths information so we need a
  // secound buffer
  size_t size = alsa2Midi( event, currentMessage );
  if ( !currentMessage.bytes.empty() ) {
    doMidiCallback( makeTimeStamp(event),
                    currentMessage.bytes.data(),
                    size );
    return true;
  } else {
#if defined( __RTMIDI_DEBUG__ )
    try {
      error( RTMIDI_ERROR( rtmidi_gettext( "Event parsing error or not an ALSA MIDI event." ),
                           Error::WARNING ) );
    } catch ( Error& e ) {
      // don't bother ALSA with an unhandled exception
    }
#endif
    return false;
  }
}
#undef RTMIDI_CLASSNAME


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
inline int AlsaMidi :: createPort( int alsaCapabilities,
                                   const std::string& portName ) {
  if ( subscription_in || subscription_out ) {
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
  snd_seq_port_info_set_name( pinfo, portName.c_str( ) );

  queue.start(seq);
  queue.preparePort(pinfo);

  int createok = seq->createPort( pinfo, static_cast<MidiAlsa *>(this) );

  if ( createok < 0 ) {
    return createok;
  }

  local.client = snd_seq_port_info_get_client( pinfo );
  local.port = snd_seq_port_info_get_port( pinfo );

  return 0;
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

inline void AlsaMidi :: deletePort () {
  queue.stop(seq);
  seq->deletePort( local.port, static_cast<MidiAlsa *>(this) );
  local.client = 0;
  local.port = 0;
}


RTMIDI_NAMESPACE_END
#endif // __LINUX_ALSA__
#endif // RTMIDI_ALSA_H
