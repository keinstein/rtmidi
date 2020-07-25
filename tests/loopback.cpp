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

// Platform-dependent sleep routines.
#if defined(WIN32)
#include <windows.h>
#define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds )
#else // Unix variants
#include <time.h>
inline void SLEEP(unsigned long long int  milliseconds ) {
  struct timespec time,time2;
  time.tv_sec = milliseconds / 1000;
  time.tv_nsec = (milliseconds % 1000) * 1000000;
  int status;
  if ((status = nanosleep(&time,&time2))) {
    int error = errno;
    std::perror("Sleep has been interrupted");
    exit(error);
  }
}
#endif

template <class T, class C = std::vector<T>, class L = std::list<C> >
class chunkedContainer {
public:
  typedef T value_type;
  typedef C chunk_type;
  typedef L list_type;
  typedef chunkedContainer<T,L,C> thistype;

  template<class I = typename list_type::iterator,
           class J = typename chunk_type::iterator,
           class V = value_type>
  class iterator {
  public:
    typedef I list_iterator;
    typedef J chunk_iterator;
    typedef V value_type;

    iterator(iterator & o)
      : outer(o.outer),
        inner(o.inner),
        inner_end(o.inner_end) {}

    iterator(iterator && o)
      : outer(std::move(o.outer)),
        inner(std::move(o.inner)),
        inner_end(std::move(o.inner_end)) {}

    iterator(list_iterator & o)
      : outer (o) {
    }
    iterator(list_iterator && o)
      : outer (o) { }
    iterator(list_iterator &&o,
             chunk_iterator & i)
      : outer (o),
        inner(i),
        inner_end(o->end()) { }
    iterator(list_iterator && o,
             chunk_iterator && i)
      : outer (o),
        inner(i),
        inner_end(o->end()) { }
    iterator & operator ++ () {
      if (++inner == inner_end) {
        inner = (++outer->begin());
        inner_end = outer->end();
      }
      return *this;
    }

    bool operator == (const iterator & o) const {
      return outer == o.outer && inner == o.inner;
    }

    bool operator != (const iterator & o) const {
      return outer != o.outer || inner != o.inner;
    }

    value_type & operator * () {
      return *inner;
    }

    value_type * operator -> () {
      return inner.operator ->();
    }

  protected:
    list_iterator outer;
    chunk_iterator inner;
    chunk_iterator inner_end;
  };
  typedef iterator<typename list_type::const_iterator,
                   typename chunk_type::const_iterator,
                   const value_type> const_iterator;

  void append(const chunk_type & c) {
    list.push_back(c);
  }
  void append(chunk_type && c) {
    list.push_back(c);
  }

  void clear() {
    list.clear();
  }

  size_t size() const {
    size_t retval = 0;
    for (auto &chunk : list) {
      retval += chunk.size();
    }
    return retval;
  }

  template<class O>
  bool operator != (const O & o) const {
    auto i1 = o.begin();
    auto e1 = o.end();
    const_iterator i2(begin());
    const_iterator e2(end());
    for(; i1 != e1  && i2 != e2; ++i1, ++i2) {
      std::cerr << "i1: " << (int)*i1 << std::endl;
      std::cerr << "i2: " << (int)*i2 << std::endl;
      if (*i1 != *i2) return false;
    }
    return (i1 == e1) && (i2 == e2);
  }
  const_iterator begin() const {
    if (list.empty())
      return const_iterator(list.begin());
    else
      return const_iterator(list.begin(),
                            list.begin()->begin());
  }
  const_iterator end() const {
    return const_iterator(list.end());
  }
protected:
  list_type list;
};


class MidiInterface : public rtmidi::SplitSysexMidiInterface {
public:
  typedef chunkedContainer<unsigned char> container;

  virtual void midiIn ( double timestamp,
                        std::vector<unsigned char>& message ) throw () {
    (void) timestamp;
    (void) message;
  }

  virtual void midiIn ( double timestamp, unsigned char * begin, ptrdiff_t size) throw() {
    (void) timestamp;
    data.append(typename container::chunk_type(begin,begin+size));
  }

  virtual void sysexChunk ( double timestamp,
                            unsigned char * begin,
                            ptrdiff_t size,
                            int connectionId ) throw() {
    (void) timestamp;
    (void) connectionId;
    data.append(typename container::chunk_type(begin,begin+size));
  }

  void clear() {
    data.clear();
  }

  size_t size() { return data.size(); }

  template <class O>
  bool operator != (const O & o) const {
    return data != o;
  }

  typename container::const_iterator begin() const {
    return data.begin();
  }
  typename container::const_iterator end() const {
    return data.end();
  }
protected:
  container data;
};

/* Here, we store the expected output. Timing is not tested */
std::vector<unsigned char> virtualinstring;
std::vector<unsigned char> virtualinstringgoal =
  {
   0xc0,  0x5, 0xf1, 0x3c, 0xb0, 0x7, 0x64, 0x90,
   0x40, 0x5a, 0x80, 0x40, 0x28, 0xb0, 0x7, 0x28,
   0xf0, 0x43, 0x4, 0x3, 0x2, 0xf7
   //"\xc0\x5\xf1\x3c\xb0\x7\x64\x90\x40\x5a\x80\x40\x28\xb0\x7\x28\xf0\x43\x4\x3\x2\xf7"
  };
std::vector<unsigned char> instring;
std::vector<unsigned char> instringgoal =
  {
   0xc0,  0x6, 0xf1, 0x3d, 0xb0, 0x8, 0x64, 0x90,
   0x41, 0x5a, 0x80, 0x41, 0x28, 0xb0, 0x8, 0x28,
   0xf0, 0x43, 0x4, 0x3, 0x3, 0xf7
  };

inline size_t getlength(const char * messages) {
	size_t retval = 0;
	const unsigned char * c = reinterpret_cast<const unsigned char *>(messages);
	while (*(c++) != 0xf7) retval++;
	return ++retval;
}

# if 0
void mycallback( double /* deltatime */, std::vector< unsigned char > *message, void * /* userData */ )
{
  std::cerr << "M+" << std::endl;
  static size_t oldsize = 0;
  if (oldsize  != instring.size()) {
    std::cerr << "Size differs: " << oldsize << " != " << instring.size() << std::endl;
    oldsize = instring.size();
  }
  unsigned int nBytes = message->size();
  instring.reserve(instring.size() + nBytes);
  for ( unsigned int i=0; i<nBytes; i++ ) {
    instring.push_back((*message)[i]);
    //		std::cout << "\\x" << std::hex << (int)message->at(i) << std::flush;
  }
  if (instring.size() != oldsize + nBytes) {
    std::cerr << "Wrong size: " << instring.size() << " != " << (oldsize + nBytes) << std::endl;
    abort();
  }
  oldsize = instring.size();
  /*	if ( nBytes > 0 )
		std::cout << "stamp = " << deltatime << std::endl;
	*/
  std::cerr << "M-" << std::endl;
}

void mycallback_virtual( double /* deltatime */, std::vector< unsigned char > *message, void * /* userData */ )
{
  std::cerr << "Mv+" << std::endl;
  static size_t oldsize = 0;
  if (oldsize  != virtualinstring.size()) {
    std::cerr << "Size differs: " << oldsize << " != " << virtualinstring.size() << std::endl;
    oldsize = virtualinstring.size();
  }
  unsigned int nBytes = message->size();
  std::cerr << "Mv*" << std::endl;
  virtualinstring.reserve(virtualinstring.size() + nBytes);
  for ( unsigned int i=0; i<nBytes; i++ ) {
    virtualinstring.push_back((*message)[i]);
    // std::cout << "\\x" << std::hex << (int)message->at(i);
  }
  std::cerr << "Mv/" << std::endl;
  if (virtualinstring.size() != oldsize + nBytes) {
    std::cerr << "Wrong size: " << virtualinstring.size() << " != " << (oldsize + nBytes) << std::endl;
    abort();
  }
  oldsize = virtualinstring.size();
	/*
	  if ( nBytes > 0 )
	  std::cout << "stamp = " << deltatime << std::endl;
	*/
  std::cerr << "Mv-" << std::endl;
}
#endif

int sysex_counter = 0;
std::vector<unsigned char> make_sysextest_message(size_t length) {
#ifdef TEST_LOOPBACK_RANDOM
#else
  static int count = 0;
#endif

  if (!length) length = 0x100000/*0*/;
  std::cout << "Generating SySex message of size " << std::dec << length << std::endl;
  std::vector<unsigned char> message(length+4);
  message[0] = 0xf0;
  if (length)
    for (size_t i = 1; i < length-1 ; ++i) {
#ifdef TEST_LOOPBACK_RANDOM
      int count = rand();
#else
      count = sysex_counter++;
#endif
      message[i]   = count & 0x7f; count >>= 7;
      message[++i] = count & 0x7f; count >>= 7;
      message[++i] = count & 0x7f; count >>= 7;
      message[++i] = count & 0x7f; count >>= 7;
      message[++i] = count & 0x7f;
    }
  message.resize(length);
  message.back() =  0xf7;
  return message;
}

void checkstrings(const MidiInterface & s1,
                  const std::vector<unsigned char> & s2) {
  size_t i=0;
  if ( s1 != s2 ) {
    std::cerr << "failed" << std::endl;
    auto i1 = s1.begin();
    auto i2 = s2.begin();
    auto e1 = s1.end();
    auto e2 = s2.end();
    for (; i1 != e1 && i2 != e2; ++i1, ++i2) {
      if (!(++i % 16)) std::cout << std::endl << std::hex << i << ": ";
      std::cout << " " << std::hex << (int) *i1;
      std::cout << "/" << std::hex << (int) *i2 << std::flush;
    }
    for (;i1 != e1;++i1) {
      if (!(++i % 16)) std::cout << std::endl << std::hex << i << ": ";
      std::cout << " " << std::hex << (int) *i1 << "/?""?" << std::flush;
    }
    for (;i2 != e2; ++i2) {
      if (!(++i % 16)) std::cout << std::endl << std::hex << i << ": ";
      std::cout << " " << "?""?/" << std::hex << (int) *i2 << std::flush;
    }
    std::cout << std::endl;
    abort();
  }
}


class abortonerror : public rtmidi :: ErrorInterface {
  void rtmidi_error ( const rtmidi :: Error & e ) {
    e.printMessage();
    switch (e.getType()) {
    case rtmidi::Error::WARNING:
    case rtmidi::Error::DEBUG_WARNING:
      break;
    case rtmidi::Error::UNSPECIFIED: /*!< The default: unspecified error type. */
    case rtmidi::Error::NO_DEVICES_FOUND: /*!< No devices found on system. */
    case rtmidi::Error::INVALID_DEVICE: /*!< An invalid device ID was specified. */
    case rtmidi::Error::MEMORY_ERROR: /*!< An error occured during memory allocation. */
    case rtmidi::Error::INVALID_PARAMETER: /*!< An invalid parameter was specified to a function. */
    case rtmidi::Error::INVALID_USE: /*!< The function was called incorrectly. */
    case rtmidi::Error::DRIVER_ERROR: /*!< A system driver error occured. */
    case rtmidi::Error::SYSTEM_ERROR: /*!< A system error occured. */
    case rtmidi::Error::THREAD_ERROR: /*!< A thread error occured. */
    default:
      abort();
    }
  }
};

int main( int /* argc */, char * /*argv*/[] )
{
  abortonerror errorhandler;
  MidiInterface inputInterface;
  MidiInterface virtualInputInterface;

  std::vector<unsigned char> message;

  try {

    // rtmidi::MidiIn constructor
    rtmidi::MidiIn virtualin;
    // rtmidi::MidiIn constructor
    rtmidi::MidiOut virtualout;

    virtualin.setErrorCallback(&errorhandler);
    virtualout.setErrorCallback(&errorhandler);

    virtualin.openVirtualPort("RtMidi Test Virtual In");
    virtualout.openVirtualPort("RtMidi Test Virtual Out");

    rtmidi::Pointer<rtmidi::PortDescriptor> indescriptor
      = virtualin.getDescriptor(true);

    rtmidi::Pointer<rtmidi::PortDescriptor> outdescriptor
      = virtualout.getDescriptor(true);

    { // avoid problems with wrong destruction order
      /* use smart pointers to handle deletion */
      rtmidi::Pointer<rtmidi::MidiInApi> midiin(outdescriptor->getInputApi());
      if (!midiin) abort();
      midiin->setErrorCallback(&errorhandler);

      rtmidi::Pointer<rtmidi::MidiOutApi> midiout(indescriptor->getOutputApi());
      if (!midiout) abort();
      midiout->setErrorCallback(&errorhandler);



      midiin->openPort(outdescriptor,
                       "RtMidi Test Real In",
                       rtmidi::PortDescriptor::INPUT);
      midiout->openPort(indescriptor,
                        "RtMidi Test Real Out",
                        rtmidi::PortDescriptor::OUTPUT);


      // Set our callback function.  This should be done immediately after
      // opening the port to avoid having incoming messages written to the
      // queue instead of sent to the callback function.
      midiin->setCallback( &inputInterface );
      virtualin.setCallback( &virtualInputInterface );

      // Don't ignore sysex, timing, or active sensing messages.
      // Don't ignore sysex, timing, or active sensing messages.
      midiin->ignoreTypes( false, false, false );
      virtualin.ignoreTypes( false, false, false );

      SLEEP( 500 );

      // Send out a series of MIDI messages.

      // Program change: 192, 5
      message.push_back( 192 );
      message.push_back( 5 );
      midiout->sendMessage( &message );
      message[1] = 6;
      virtualout.sendMessage(&message);

      SLEEP( 500 );

      // Quarter frame
      message[0] = 0xF1;
      message[1] = 60;
      midiout->sendMessage( &message );
      message[1] = 61;
      virtualout.sendMessage(&message);

      // Control Change: 176, 7, 100 (volume)
      message[0] = 176;
      message[1] = 7;
      message.push_back( 100 );
      midiout->sendMessage( &message );
      message[1] = 8;
      virtualout.sendMessage ( &message );

      // Note On: 144, 64, 90
      message[0] = 144;
      message[1] = 64;
      message[2] = 90;
      midiout->sendMessage( &message );
      message[1] = 65;
      virtualout.sendMessage( &message );

      SLEEP( 500 );

      // Note Off: 128, 64, 40
      message[0] = 128;
      message[1] = 64;
      message[2] = 40;
      midiout->sendMessage( &message );
      message[1] = 65;
      virtualout.sendMessage( &message );

      SLEEP( 500 );

      // Control Change: 176, 7, 40
      message[0] = 176;
      message[1] = 7;
      message[2] = 40;
      midiout->sendMessage( &message );
      message[1] = 8;
      virtualout.sendMessage( &message );

      SLEEP( 500 );

      // Sysex: 240, 67, 4, 3, 2, 247
      message[0] = 240;
      message[1] = 67;
      message[2] = 4;
      message.push_back( 3 );
      message.push_back( 2 );
      message.push_back( 247 );
      midiout->sendMessage( &message );
      message[4] = 3;
      virtualout.sendMessage( &message );

      SLEEP( 500 );

      std::cout << "Virtual output -> input:" << std::endl;
      std::cout << "Sent " << instringgoal.size() << " bytes, got " << inputInterface.size();
      checkstrings(inputInterface, instringgoal);


      std::cout << "Output -> virtual input:" << std::endl;
      std::cout << "Sent " << virtualinstringgoal.size() << " bytes, got " << virtualInputInterface.size();
      checkstrings(virtualInputInterface, virtualinstringgoal);

      size_t maxsysexsize = midiout->maxSysExSize( );
      std::cout << "midiout->maxSysExSize( )   = " << maxsysexsize << std::endl;
      std::vector<unsigned char>
        longsysex1 = make_sysextest_message( maxsysexsize );

      maxsysexsize = virtualout.maxSysExSize( );
      std::cout << "virtualout.maxSysExSize( ) = " << maxsysexsize << std::endl;
      std::vector<unsigned char>
        longsysex2 = make_sysextest_message( maxsysexsize );
      inputInterface.clear();
      virtualInputInterface.clear();
      midiout -> sendMessage( longsysex1 );
      virtualout.sendMessage( longsysex2 );

      SLEEP(500);

      std::cout << "Long virtual input" << std::endl;
      checkstrings(virtualInputInterface, longsysex1);
      std::cout << "Long input" << std::endl;
      checkstrings(inputInterface,        longsysex2);
    }

  } catch ( rtmidi::Error &error ) {
    error.printMessage();
    abort();
  }
  return 0;
}
