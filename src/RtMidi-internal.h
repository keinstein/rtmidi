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

#ifndef RTMIDI_INTERNAL_H
#define RTMIDI_INTERNAL_H

#include "RtMidi.h"
#include <sstream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <functional>
#include <cerrno>
#include <mutex>
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


#define RTMIDI_CLASSNAME "InternalMidiApi"
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
    if ( userCallback ) {
      userCallback->midiIn( timestamp, buffer, size );
      return true;
    }
    return false;
  }

  bool doSysExCallback( double timestamp,
                        unsigned char * buffer,
                        ptrdiff_t size,
                        int connectionId ) {
    if ( userCallback ) {
      userCallback->sysexChunk( timestamp, buffer, size, connectionId );
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
                 unsigned int capabilities) {
    openPort(getPort(portNumber,capabilities),
             portName,
             capabilities);
  }
  using MidiApi::getPortName;
  std::string getPortName ( unsigned int portNumber,
                            unsigned int capabilities ) {
    return getPort(portNumber, capabilities)->getName();
  }

  bool firstMessage;
  bool continueSysex;
protected:
  char ignoreFlags;
  SplitSysexMidiInterface * userCallback;

  virtual PortPointer getPort(unsigned int portNumber,
                              unsigned int capabilities) {
    PortList list = getPortList(capabilities);
    if (portNumber >= list.size()) {
      error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                            Error::INVALID_PARAMETER, portNumber ) );
    }
    for (auto & port : list) {
      if (!portNumber--) {
        return port;
      }
    }
    error( RTMIDI_ERROR1( gettext_noopt( "The 'portNumber' argument ( %d ) is invalid." ),
                          Error::INVALID_PARAMETER, portNumber ) );
    return nullptr;
  }
};
#undef RTMIDI_CLASSNAME



// Define API names and display names.
// Must be in same order as API enum.
extern "C"
  struct api_name {
    ApiType Id;
    const char * machine;
    const char * display;
  };


// Flags for receiving MIDI
enum ignore_type {
      IGNORE_SYSEX = 0x01,
      IGNORE_TIME = 0x02,
      IGNORE_SENSING = 0x04
};
extern ignore_type ignore_flags;

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

#if 1
class testmutex: public std::mutex {
public:
  testmutex(const std::string & l = "*",
            const std::string & u = "/",
            const std::string & t = "!",
            const std::string & n = "-")
    : mutex() {
    rtmidiUnused(l);
    rtmidiUnused(u);
    rtmidiUnused(t);
    rtmidiUnused(n);
  }
};
#else
class testmutex: public std::mutex {
public:
  testmutex(const std::string & l = "*",
            const std::string & u = "/",
            const std::string & t = "!",
            const std::string & n = "-")
    : mutex(),
      locks(l),
      unlocks(u),
      tlocks(t),
      ntlocks(n) {}
  void lock() {
    std::mutex::lock();
    std::cout << locks << std::endl;
  }
  void unlock() {
    std::cout << unlocks << std::endl;
    std::mutex::unlock();
  }
  bool try_lock() {
    bool retval = std::mutex::try_lock();
    std::cout << (retval?tlocks:ntlocks) << std::endl;
    return retval;
  }
  std::string locks, unlocks, tlocks, ntlocks;
};
#endif

#define RTMIDI_CLASSNAME "SequencerPortList"
template <class P>
struct SequencerPortList {
  typedef P PortType;
  typedef std::lock_guard<SequencerPortList<PortType> > lock_guard;
  struct item {
    PortType * port;
    std::atomic<item *> next=nullptr;

    item() = delete;
    item (P * p):port(p),next(nullptr) {}
  };

  class ItemIterator {
  public:
    ItemIterator (SequencerPortList & list)
      : i (list.first) {}

    ItemIterator () : i(nullptr) {}

    ItemIterator & operator ++ () {
      if (i) i = i -> next;
      return *this;
    }

    bool operator != (const ItemIterator & o) {
      return i != o.i;
    }

    item* operator-> () {
      return i;
    }
  protected:
    item * i;
  };

  class LockedItemIterator : public ItemIterator {
  public:
    LockedItemIterator (SequencerPortList & list)
      : ItemIterator (list), lock(list) {}

    LockedItemIterator & operator ++ () {
      ++(static_cast<ItemIterator &>(*this));
      return *this;
    }

    item* operator -> () {
      return ItemIterator::operator->();
    }

    bool operator == (const ItemIterator & o) {
      return ItemIterator::operator == (o);
    }
  protected:
    std::lock_guard<SequencerPortList> lock;
  };

  SequencerPortList()
    : m(),
      write_mutex("w","x","y","z"),
      first(nullptr)
  {}

  ItemIterator begin() {
    return ItemIterator(*this);
  }

  ItemIterator end() {
    return ItemIterator();
  }

  void add (PortType * port) {
    std::lock_guard<testmutex> guard(write_mutex);
    auto n = new item(port);
    {
      std::atomic_thread_fence(std::memory_order_acq_rel);
      last->store(n, std::memory_order_acq_rel);
    }
    last = &(n->next);
  }

  void remove (PortType * port) {
    std::lock_guard<testmutex> guard(write_mutex);
    for (auto  i = &first; *i ; i = &(i->load())->next) {
      if ((i->load())->port == port) {
        item * save = *i;
        {
          // lock as short as possible to avoid breaking real time operations
          std::lock_guard<testmutex> guard(m);
          i->store(save->next);
        }
        // ensure last points to the right position
        for (last = &first; *last ; last = &(last->load())->next);
        delete save;
        break;
      }
    }
  }

  void lock() {
    write_mutex.lock();
  }

  void unlock() {
    write_mutex.unlock();
  }


  bool operator != (const ItemIterator & o) {
    return ItemIterator::operator !=(o);
  }
protected:
  /**
   * Mutex to guard reads.
   * We can't delete any port object while the realtime thread may use it.
   *
   */
  testmutex m;
  /**
   * Mutex to guards writes.  This is vor convenience when several
   * threads try to create or remove ports.
   *
   */
  testmutex write_mutex;
  std::atomic<item *> first, *last=&first;
};
#undef RTMIDI_CLASSNAME


#define RTMIDI_CLASSNAME "CopyOnWrite"
template <class C>
class CopyOnWrite {
public:
  typedef C container;
  typedef typename container::value_type value_type;
  typedef typename container::const_iterator nonlocked_const_iterator;
  typedef std::lock_guard<CopyOnWrite> lock_guard;
  typedef std::lock_guard<const CopyOnWrite> const_lock_guard;

  class locked_iterator:protected container::const_iterator {
  public:
    locked_iterator(const CopyOnWrite<container> & c)
      : container::const_iterator(c.data.begin()),
        lock(c) {}

    bool operator == (typename container::const_iterator & o) {
      return static_cast<typename container::const_iterator &>(*this) == o;
    }

    bool operator != (typename container::const_iterator & o) {
      return static_cast<typename container::const_iterator &>(*this) != o;
    }

    locked_iterator & operator ++ () {
      ++static_cast<typename container::const_iterator &>(*this);
      return *this;
    }

    using container::const_iterator::operator *;
  protected:
    const_lock_guard lock;
  };

  CopyOnWrite()
    : data(),
      mutex() {}

  CopyOnWrite(const container & c)
    : data(c),
      mutex() {}

  CopyOnWrite(container & c)
    : data(c),
      mutex() {}

  CopyOnWrite(container && c)
    : data(c),
      mutex() {}

  locked_iterator begin() { return locked_iterator(*this); }
  typename container::const_iterator end() { return data.end(); }

  void lock() const { const_cast<std::mutex&>(mutex).lock(); }
  void unlock() const { const_cast<std::mutex&>(mutex).unlock(); }

  void swap(container && c) {
    swap( c, internal_lock_guard( this->write_mutex ) );
  }

  container getData () {
    return getData(internal_lock_guard(write_mutex));
  }
  void add (size_t number, const value_type item) {
    internal_lock_guard lock(write_mutex) ;
    container c = getData(lock);
    if (c.size() <= number) c.resize(number+1);
    c[number]=item;
    swap(std::move(c),lock);
  }

  void remove (const value_type & item) {
    internal_lock_guard lock(write_mutex) ;
    container c = getData(lock);
    auto it = std::find(c.begin(),c.end(),item);
    if (it != c.end()) {
      *it = nullptr;
      swap(std::move(c),lock);
    }
  }

  size_t size() const { return data.size(); }
  value_type & operator[] (size_t nr) { return data[nr]; }
  const value_type & operator[] (size_t nr) const { return data[nr]; }
protected:
  typedef std::lock_guard<std::mutex> internal_lock_guard;
  container data;
  std::mutex mutex, write_mutex;

  void swap(container && c,
            const internal_lock_guard & write_lock) {
    (void) write_lock;
    internal_lock_guard lock(mutex);
    data.swap(c);
  }

  container getData (const internal_lock_guard & write_lock ) {
    (void) write_lock;
    container retval(data);
    return retval;
  }

};
#undef RTMIDI_CLASSNAME


RTMIDI_NAMESPACE_END
#endif // RTMIDI_INTERNAL_H
