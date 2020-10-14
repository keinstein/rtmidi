/********************* -*- C++ -*- ****************************************/
/*! \class RtMidi
  \brief An abstract base class for realtime MIDI input/output.

  This class implements some common functionality for the realtime
  MIDI input/output subclasses RtMidiIn and RtMidiOut.

  RtMidi WWW site: http://music.mcgill.ca/~gary/rtmidi/

  RtMidi: realtime MIDI i/o C++ classes
  Copyright ( c ) 2003-2017 Gary P. Scavone
  Forked by Tobias Schlemmer, 2014-2018.

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation files
  ( the "Software" ), to deal in the Software without restriction,
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

/*!
  \file RtMidi.h
*/

#ifndef RTMIDI_H
#define RTMIDI_H

#define RTMIDI_VERSION "4.0.0"

#ifdef RTMIDI_NO_WARN_DEPRECATED
#define RTMIDI_DEPRECATED( func, message ) func
#else
#ifdef __GNUC__
#define RTMIDI_DEPRECATED( func, message ) func __attribute__ ( ( deprecated ( message ) ) )
#elif defined ( _MSC_VER )
#define RTMIDI_DEPRECATED( func, message ) __declspec ( deprecated ( message ) ) func
#else
#pragma message ( "WARNING: You need to implement the macro RTMIDI_DEPRECATED for this compiler if this code doesn't compile" )
#define RTMIDI_DEPRECATED( func, message ) func [[deprecated ( message ) ]]
#endif
#endif

#define rtmidiUnused( x ) do { ( void ) x; } while ( 0 )

// Check for C++11 support
#if defined ( _MSC_VER ) && _MSC_VER >= 1800
// At least Visual Studio 2013
#define RTMIDI_SUPPORTS_CPP11 1
#elif __cplusplus >= 201103L
#define RTMIDI_SUPPORTS_CPP11 1
#else
#define RTMIDI_SUPPORTS_CPP11 0
#endif

#if defined _WIN32 || defined __CYGWIN__
#if defined ( RTMIDI_EXPORT )
#define RTMIDI_DLL_PUBLIC __declspec ( dllexport )
#else
#define RTMIDI_DLL_PUBLIC
#endif
#else
#if __GNUC__ >= 4
#define RTMIDI_DLL_PUBLIC __attribute__ ( ( visibility ( "default" ) ) )
#else
#define RTMIDI_DLL_PUBLIC
#endif
#endif

// some fake libintl implementations may need special headers.
#ifdef RTMIDI_GETTEXT
#include "libintl.h"
#endif


#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <memory>
#include <stdexcept>
#include <atomic>
// the following are used in the error constructor
#include <cstdarg>
#include <cstddef>
#include <cstring>
#include <cstdio>


#define gettext_noopt( str ) ( str )

#define RTMIDI_NAMESPACE_START namespace rtmidi {
#define RTMIDI_NAMESPACE_END }

RTMIDI_NAMESPACE_START
#ifdef RTMIDI_GETTEXT
void init_rtmidi_gettext ( );
RTMIDI_DLL_PUBLIC const char * rtmidi_gettext ( const char * s );
#else
#define rtmidi_gettext( arg ) ( arg )
#endif


//! MIDI API specifier arguments.
enum ApiType {
              UNSPECIFIED, /*!< Search for a working compiled API. */
              MACOSX_CORE, /*!< Macintosh OS-X Core Midi API. */
              LINUX_ALSA, /*!< The Advanced Linux Sound Architecture API. */
              UNIX_JACK, /*!< The JACK Low-Latency MIDI Server API. */
              WINDOWS_MM, /*!< The Microsoft Multimedia MIDI API. */
              DUMMY, /*!< A compilable but non-functional API. */
              WINDOWS_KS, /*!< The Microsoft Kernel Streaming MIDI API. */
              ALL_API, /*!< Use all available APIs for port selection. */
              NUM_APIS /*!< Number of values in this enum. */
};


// A MIDI structure used internally by the class to store incoming
// messages. Each message represents one and only one MIDI message.
struct RTMIDI_DLL_PUBLIC MidiMessage {
  std::vector<unsigned char> bytes;
  //! Time in seconds elapsed since the previous message
  double timeStamp;

  // Default constructor.
  MidiMessage ( )
    : bytes ( 0 ), timeStamp ( 0.0 ) {}
  MidiMessage ( size_t size )
    : bytes ( size ), timeStamp ( 0.0 ) {}
};

struct RTMIDI_DLL_PUBLIC MidiQueue {

  // Default constructor.
  MidiQueue ( )
    : front ( 0 ), back ( 0 ), ringSize ( 0 ), ring ( nullptr ) {}
  //! Constructor
  /**
   *
   *
   * \param size Number of messages in the MIDI queue
   */
  MidiQueue ( size_t size )
    : front ( 0 ), back ( 0 ), ringSize ( size ) {
    if ( ringSize > 0 )
      ring = new MidiMessage[ ringSize ];
  }

  ~MidiQueue ( void ) {
    if (ring) delete [] ring;
  }

  bool push ( const MidiMessage& );
  /**
   * Pop a message from the queue. The function returns true if a
   * message could be retrieved, false otherwise.
   *
   * \param message A vector that is used to store the message
   * \param timestamp Timestamp when the message arrived.
   *
   * \retval true  a message could be retrieved.
   * \retval false the queue is empty
   */
  bool pop ( std::vector<unsigned char>& message, double& timestamp );

  /**
   * Check whether the queue is empty.
   *
   *
   * \retval true if the queue is empty
   * \retval false if the queue contains elements.
   */
  bool empty () {
    return front == back;
  }
protected:
  unsigned int size ( unsigned int * back=0,
                      unsigned int * front=0 );
  unsigned int front;
  unsigned int back;
  unsigned int ringSize;
  MidiMessage * ring;

};



/************************************************************************/
/*! \class Error
  \brief Exception handling class for RtMidi.

  The Error class is quite simple but it does allow errors to be
  "caught" by Error::Type. See the RtMidi documentation to know
  which methods can throw an Error.
*/
/************************************************************************/

#define RTMIDI_ERROR( message, type )                                \
  rtmidi::Error ( message, type,                                      \
                  RTMIDI_CLASSNAME, __FUNCTION__,                     \
                  __FILE__, __LINE__ )
#define RTMIDI_ERROR1( message, type, arg1 )                         \
  rtmidi::Error ( message, type,                                      \
                  RTMIDI_CLASSNAME, __FUNCTION__,                     \
                  __FILE__, __LINE__, arg1 )
#define RTMIDI_ERROR2( message, type, arg1, arg2 )                     \
  rtmidi::Error ( message, type,                                      \
                  RTMIDI_CLASSNAME, __FUNCTION__,                     \
                  __FILE__, __LINE__, arg1, arg2 )

class RTMIDI_DLL_PUBLIC Error : public std::exception
{
public:
  //! Defined Error types.
  enum Type {
             WARNING, /*!< A non-critical error. */
             DEBUG_WARNING, /*!< A non-critical error which might be useful for debugging. */
             UNSPECIFIED, /*!< The default, unspecified error type. */
             NO_DEVICES_FOUND, /*!< No devices found on system. */
             INVALID_DEVICE, /*!< An invalid device ID was specified. */
             MEMORY_ERROR, /*!< An error occured during memory allocation. */
             INVALID_PARAMETER, /*!< An invalid parameter was specified to a function. */
             INVALID_USE, /*!< The function was called incorrectly. */
             DRIVER_ERROR, /*!< A system driver error occured. */
             SYSTEM_ERROR, /*!< A system error occured. */
             THREAD_ERROR /*!< A thread error occured. */
  };

  //! The constructor.
  Error ( const char * message,
          Type type,
          const char * class_name,
          const char * function_name,
          const char * file_name,
          int line_number, ... ) throw ( );

  //! The destructor.
  virtual ~Error ( void ) throw ( ) {}

  //! Prints thrown error message to stderr.
  virtual void printMessage ( std::ostream& s = std::cerr ) const throw ( ) {
    s << std::endl
      << file << ":" << line << ": in function "
      << classname << "::" << function << std::endl
      << message_ << std::endl << std::endl;
  }

  //! Returns the thrown error message type.
  virtual const Type& getType ( void ) const throw ( ) { return type_; }

  //! Returns the thrown error message string.
  virtual const std::string& getMessage ( void ) const throw ( ) { return message_; }

  //! Returns the thrown error message as a c-style string.
  virtual const char * what ( void ) const throw ( ) { return message_.c_str ( ); }

protected:
  const char * classname;
  const char * function;
  const char * file;
  int line;
  std::string message_;
  Type type_;
};

struct ErrorInterface {
  virtual ~ErrorInterface ( ) {}
  virtual void rtmidi_error ( const Error & e ) = 0;
  virtual void deleteMe ( ) {};
};

#if !RTMIDI_SUPPORTS_CPP11
class PortDescriptor;

template<class T>
class Pointer {
public:
  typedef T datatype;
protected:
  struct countPointer {
    int count;
    datatype * descriptor;
  };
public:
  Pointer ( )
    : ptr ( 0 ) {}
  Pointer ( datatype * p )
    : ptr ( new countPointer ) {
    ptr->count = 1;
    ptr->descriptor = p;
  }
  Pointer ( const Pointer<datatype>& other )
    : ptr ( other.ptr ) {
    if ( ptr )
      ptr->count++;
  }
  Pointer ( const Pointer<datatype>&& other )
    : ptr ( other.ptr ) {
  }

  ~Pointer ( ) {
    if ( !ptr ) return;
    if ( ! ( -- ( ptr->count ) ) ) {
      delete ptr->descriptor;
      delete ptr;
    }
  }

  datatype * operator -> ( ) {
    if ( !ptr ) return 0;
    // this should throw an exception

    return ptr->descriptor;
  }

  datatype& operator * ( ) {
    if ( !ptr || !ptr->descriptor ) {
      throw std::invalid_argument ( "rtmidi::Pointer: trying to dereference a NULL pointer." );
    }
    else return ( *ptr->descriptor );
  }

  const datatype& operator * ( ) const {
    if ( !ptr || !ptr->descriptor ) {
      throw std::invalid_argument ( "rtmidi::Pointer: trying to dereference a NULL pointer." );
    }
    else return ( *ptr->descriptor );
  }

  bool operator ! ( ) {
    return ( !ptr || !ptr->descriptor );
  }

  operator bool ( ) {
    return ( ptr && ptr->descriptor );
  }

  Pointer& operator = ( const Pointer<datatype>& other ) {
    if ( ptr ) {
      if ( ! ( --ptr->count ) ) {
        delete ptr->descriptor;
        delete ptr;
      }
    }
    if ( ( ptr = other.ptr ) )
      ptr->count++;
    return *this;
  }
protected:
  countPointer * ptr;
};

template <class T, class U>
bool operator== ( const Pointer<T>& lhs, const Pointer<U>& rhs ) {
  return ( & ( *lhs ) ) == ( & ( *rhs ) );
}

template <class T, class U>
bool operator!= ( const Pointer<T>& lhs, const Pointer<U>& rhs ) {
  return ( & ( *lhs ) ) != ( & ( *rhs ) );
}

#else
template<class T>
using Pointer = std::shared_ptr<T>;
#endif

class MidiApi;
typedef MidiApi MidiInApi;
typedef MidiApi MidiOutApi;
typedef Pointer<MidiApi> MidiApiPtr;
typedef std::list <MidiApiPtr> MidiApiList;

class PortDescriptor {
public:
  //! Flags for formatting a string description of the port.
  /*! These flags just mark the requirements that the string
    should fulfil. An API may return the same string for
    different requirements e.g. the same short and long
    name. */
  enum NamingType: unsigned int {
                   SHORT_NAME = 0, /*!< A short human readable name
                                     depending on the API
                                     e.g. “Ensoniq AudioPCI” */
                   LONG_NAME, /*!< A complete human readable
                                name depending on the API
                                e.g. "Ensoniq AudioPCI: ES1371" */
                   SESSION_PATH, /*!< A unique description that can
                                   be used to identify the port
                                   during runtime. It may be a
                                   cryptic string. */
                   STORAGE_PATH, /*!< A unique description that is
                                   optimised for storage in
                                   configuration files. This is a
                                   more textual representation that
                                   is more robust to small changes in
                                   the surrounding environment than
                                   \ref SESSION_PATH */
                   NAMING_MASK = 0x0F, /*!< part of the flags
                                         that is concerned with
                                         naming.
                                       */
                   UNIQUE_PORT_NAME = 0x10, /*!< Make all names uniqe. This
                                              is usually done by adding
                                              numbers to the end of the
                                              string \note: use #undef UNIQUE_PORT_NAME
                                              on windows in case of any errors */
                   INCLUDE_API = 0x20 /*!< Add a string describing the
                                        API at the beginning of the
                                        string. */
  };

  //! Flags describing the capabilities of a given port.
  enum PortCapabilities: unsigned int {
                         INPUT = 1, /*!< Ports that can be read from. */
                         OUTPUT = 2, /*!< Ports that can be written to. */
                         INOUTPUT = 3, /*!< Ports that allow reading and writing ( INPUT | OUTPUT ) */
                         VIRTUAL = 4,  /*!< Virtual ports, this is mainly used for creating Api objects */
                         UNLIMITED = 0x10 /*!< Some APIs can filter out certain ports which they consider
                                            not to be useful. This flags supresses this behaviour and
                                            selects all ports that are useable. */
  };

  //! Default constructor.
  /*!
   * Derived classes should have a constructor.
   */
  PortDescriptor ( ) {};

  //! A virtual destructor
  /*! As we might have to destruct the object from the application code
   * each port id must have a virtual destructor.
   */
  virtual ~PortDescriptor ( ) {};


  //! Get the MIDI api for the current port.
  /*! RtMidi needs to know: Which API should handle this object and in
   * which direction the data shall be transferred. This can be used
   * to get an API which can do exactly what we want..
   *
   * \param capabilities Determine whether the port shall be used for
   * input, output, both or somthing other.
   *
   * \return API that can use this object to connect or 0 if no input
   * API can be created.
   */
  virtual MidiApi * getApi ( unsigned int capabilities ) const = 0;

  //! Get the MIDI input api for the current port.
  /*! This is the only information RtMidi needs to know: Which
   * API should handle this object. This can be used to get
   * an API which can send data to the given port.
   *
   * \return API that can use this object to connect to an input port or 0
   * if no input API can be created.
   */
  virtual MidiInApi * getInputApi ( ) const;

  //! Get the MIDI output api for the current port.
  /*! This is the only information RtMidi needs to know: Which
   * API should handle this object. This can be used to get
   * an API which can receive data from the given port.
   *
   * \return API that can use this object to connect to an output port or 0
   * if no output API can be created.
   */
  virtual MidiOutApi * getOutputApi ( ) const;

  //! Return the port name
  /*!
   * \param flags A description of the requirements of the returned name.
   * \return A name that is formatted according to \ref flags.
   * \sa NamingTypes
   */
  virtual std::string getName ( int flags = SHORT_NAME | UNIQUE_PORT_NAME ) = 0;

  //! Get capabilities
  /*! \return a capabilities flag describing the capabilities of the port.
   * \sa PortCapabilities
   */
  virtual int getCapabilities ( ) const = 0;

  //! Compare two PortDescriptors
  /*! \return true if both descriptors describe the same port
   */
  virtual bool operator == ( const PortDescriptor& o ) = 0;
};

//! A list of port descriptors.
/*! Port descriptors are stored as shared pointers. This avoids
  unnecessary duplication of the data structure and handles automatic
  deletion if all references have been removed. */
typedef Pointer<PortDescriptor> PortPointer;
typedef std::list<PortPointer> PortList;


//! C++ style user callback interface.
/*!
  This interface class can be used to implement type safe MIDI callbacks.

  In this version every time a chunk of a SysEx messages is received
  the facntion #sysexChunk is called. in order to process it. It is
  recommended to use this interface whenever possible.

  Every time a MIDI message is received the function \ref
  MidiInterface::rtmidi_midi_in of the currently set callback object
  is called.
*/
struct RTMIDI_DLL_PUBLIC SplitSysexMidiInterface {
  //! Virtual destructor to avoid unexpected behaviour.
  virtual ~SplitSysexMidiInterface ( ) {}

  //! The MIDI callback function.
  /*! This function is called whenever a MIDI packet is received by a
    MIDI backend that uses the corresponding callback object.
    \param timestamp the timestamp when the MIDI message has been received
    \param message the message itself.
  */
  virtual void midiIn ( double timestamp, std::vector<unsigned char>& message ) throw () = 0;

  virtual void midiIn ( double timestamp, unsigned char * begin, ptrdiff_t size) throw() {
    std::vector<unsigned char> message(begin,begin+size);
    midiIn(timestamp, message);
  }

  //! Receive a chunk of a system exclusive message.
  /**
   * System exclusive messages are delviered in chunks in some
   * setups. The first chunk always start with 0xf0, while the last
   * one ends with 0xf7. All other chunks are considered to be
   * intermediate chunks.
   *
   * \param timestamp Time when the chunk was received.
   * \param begin Pointer to the first character of the chunk.
   * \param size Length of the chunk in characters.
   * \param connectionId Id of the sending interface. As some
   * interfaces allow several connections to a certain port, this variable can help to
   * deal with interleaving packets.
   */
  virtual void sysexChunk ( double timestamp,
                            unsigned char * begin,
                            ptrdiff_t size,
                            int connectionId ) throw() = 0;

  //! Delete the object if necessary.
  /*! This function allows the user to delete the Midi callback object,
    when MIDI backend drops its reference to it. By default it does nothing.
    But, callback objects are owned by the MIDI backend. These must be deleted
    after the reference to them has been dropped.

    \sa CompatibilityMidiInterface
  */
  virtual void deleteMe ( ) {}
};


//! C++ style user callback interface.
/*!
  The standard collects all SysEx data


  This interface class can be used to implement type safe MIDI callbacks.
  Every time a MIDI message is received the function \ref MidiInterface::rtmidi_midi_in of the
  currently set callback object is called.
*/
struct RTMIDI_DLL_PUBLIC MidiInterface : public SplitSysexMidiInterface {
  //! Virtual destructor to avoid unexpected behaviour.
  virtual ~MidiInterface ( ) {}

  //! Reconstruct a system exclusive message.
  /**
   * This function collects all chunks of a system exclusive message
   * and as soon as the message is completed, #midiIn gets called.
   *
   * \param timestamp Time when the chunk was received.
   * \param begin Pointer to the first character of the chunk.
   * \param size Length of the chunk in characters.
   * \param connectionId Id of the sending interface. As some
   * interfaces allow several connections to a certain port, this variable can help to
   * deal with interleaving packets.
   */
  virtual void sysexChunk ( double timestamp,
                            unsigned char * begin,
                            ptrdiff_t size,
                            int connectionId ) throw();

};


//! A callback class for implementing MIDI Queues.
/**
 * This class implements queue based MIDI Input. It can be set as
 * callback object via Midi::setCallback (MidiInterface *). It
 * replaces the MIDI queue form RtMidiIn which has been removed in
 * order to simplify API management.
 */
struct RTMIDI_DLL_PUBLIC MidiQueueInterface: public MidiInterface {
  //! Virtual constructor to avoid unexpected behaviour.
  MidiQueueInterface ( size_t size ):queue(size) {}

  //! Virtual destructor to avoid unexpected behaviour.
  virtual ~MidiQueueInterface (  ) {}

  //! The MIDI callback function.
  /*! This function is called whenever a MIDI packet is received by a
    MIDI backend that uses the corresponding callback object.
    \param timestamp the timestamp when the MIDI message has been received
    \param message the message itself.
  */
  virtual void midiIn ( double timestamp, std::vector<unsigned char>& message ) throw() {
    MidiMessage msg (message.size());
    std::swap(message, msg.bytes);
    msg.timeStamp = timestamp;
    queue.push(msg);
  }

  //! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
  /*!
    This function returns immediately whether a new message is
    available or not. A valid message is indicated by a non-zero
    vector size. An exception is thrown if an error occurs during
    message retrieval or an input connection was not previously
    established.

    \param message a vector that will be filled with the message.
  */
  double getMessage ( std::vector<unsigned char>& message ) {
    double retval;
    if (!queue.pop(message,retval))
      message.clear();
    return retval;
  }

  /**
   * Check whether the queue is empty.
   *
   *
   * \retval true if the queue is empty
   * \retval false if the queue contains elements.
   */
  bool empty () {
    return queue.empty();
  }

  //! Delete the object if necessary.
  /*! This function allows the user to delete the Midi callback object,
    when MIDI backend drops its reference to it. By default it does nothing.
    But, callback objects are owned by the MIDI backend. These must be deleted
    after the reference to them has been dropped.

    \sa CompatibilityMidiInterface
  */
  virtual void deleteMe ( ) {  }
protected:
  MidiQueue queue;
};

/*! \class Midi
  \brief A global class that implements basic backend API handling.

  This class enhances \ref MidiApi by some functionality to handle
  backend API objects. It serves as base class for the public RtMidi
  API.

  by Gary P. Scavone, 2003-2014.
*/
/**********************************************************************/
#define RTMIDI_CLASSNAME "Midi"
struct RTMIDI_DLL_PUBLIC Midi {
 public:
 Midi ( ApiType api=rtmidi::UNSPECIFIED,
        unsigned int capabilities=rtmidi::PortDescriptor::INOUTPUT,
        const std::string& clientName = std::string ( "RtMidi Input Client" ),
        bool pfsystem = true );

 //! A static function to determine the current RtMidi version.
 static std::string getVersion ( void ) throw ( );

 //! A virtual function that returns the capabilities associated with this class


 //! A static function to determine the available compiled MIDI APIs.
 /*!
   The values returned in the std::vector can be compared against
   the enumerated list values. Note that there can be more than one
   API compiled for certain operating systems.

   \param apis A vector apis must be provided for the
   return value. All data in this vector will be
   deleted prior to filling in the API data.

   \param preferSystem An opitonal boolean parameter
   may be provided that tells wheter system or software
   APIs shall be prefered. Passing \c true will prefer OS provided APIs
 */
 static void getCompiledApi ( std::vector<ApiType>& apis, bool preferSystem = true ) throw ( );

 //! A static function to determine the available compiled MIDI APIs.
 /*!
   The values returned in the std::vector can be compared against
   the enumerated list values. Note that there can be more than one
   API compiled for certain operating systems.

   \param preferSystem An opitonal boolean parameter
   may be provided that tells wheter system or software
   APIs shall be prefered. Passing \c true will prefer OS provided APIs
 */
 static std::vector<ApiType> getCompiledApi ( bool preferSystem = true ) throw ( );

 //! Return the name of a specified compiled MIDI API.
 /*!
   This obtains a short lower-case name used for identification purposes.
   This value is guaranteed to remain identical across library versions.
   If the API is unknown, this function will return the empty string.
 */
 static std::string getApiName ( ApiType api );

 //! Return the display name of a specified compiled MIDI API.
 /*!
   This obtains a long name used for display purposes.
   If the API is unknown, this function will return the empty string.
 */
 static std::string getApiDisplayName ( ApiType api );

 //! Return the compiled MIDI API having the given name.
 /*!
   A case insensitive comparison will check the specified name
   against the list of compiled APIs, and return the one which
   matches. On failure, the function returns UNSPECIFIED.
 */
 static ApiType getCompiledApiByName ( const std::string& name, bool preferSystem = true );

 //! Returns the MIDI API specifier for the current instance of rtmidi::MidiIn.
 ApiType getCurrentApi ( void ) throw ( );

 //! Returns true if we can open virtual ports;
 bool hasVirtualPorts ( );

 //! Returns the largest number of bytes a system exclusive message can have ;
 size_t maxSysExSize ( ) const;


 //! Returns true if a port is open and false if not.
 /*!
   Note that this only applies to connections made with the openPort ( )
   function, not to virtual ports.

   \retval true if a port is open and
   \retval false if the port is not open ( e.g. not opend or closed ) .
 */
 bool isPortOpen ( void ) const;

 //! Function to create a virtual port, with optional name.
 /*!
   This function creates a virtual MIDI port to which other
   software applications can connect. This type of functionality
   is currently only supported by the Macintosh OS-X, any JACK,
   and Linux ALSA APIs ( the function returns an error for the other APIs ) .

   If the port is not available for this type of connection, an exception is thrown.

   \param portName An optional name for the application port that is
   used to connect to portId can be specified.
   \param capabilities The kind of connection we shall establish.
 */
 void openVirtualPort ( const std::string& portName,
                        unsigned int capabilities);

 //! Open a MIDI connection given by a port descriptor.
 /*!
   If the port is not available for this type of connection, an exception is thrown.

   \param port A port descriptor of the port must be specified.
   \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
   \param capabilities The kind of connection we shall establish.

 */
 void openPort ( const PortDescriptor& port,
                 const std::string& portName,
                 unsigned int capabilities );

 //! Open a MIDI connection given by a port descriptor pointer.
 /*!
   If the port is not available for this type of connection, an exception is thrown.

   \param port A pointer to a port descriptor of the port must be specified.
   \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
   \param capabilities The kind of connection we shall establish.
   \sa openPort ( const PortDescriptor &, const std::string& );
 */
 void openPort ( Pointer<PortDescriptor> port,
                 const std::string& portName,
                 unsigned int capabilities );


 //! Close an open MIDI connection ( if one exists ) .
 void closePort ( void );

 //! Pure virtual function to return a port descirptor if the port is open
 Pointer<PortDescriptor> getDescriptor ( bool local=false );

 //! Return a list of all available ports of the current API.
 /*!
   \param capabilities an optional parameter that
   describes which capabilities the returned devices
   must support. The returned devices may have
   additional capabilities to those which have been
   requested, but not less.

   \return This function returns a list of port descriptors.

   \note Each API will request additonal
   capabilites. An output API will set always add \ref
   PortDescriptor::OUTPUT to the mask while an input
   device will always add \ref PortDescriptor::OUTPUT.

   \note An input API may but need not necessarily
   report output devices which cannot be used as input
   if \ref PortDescriptor::OUTPUT is passed as \ref
   capabilities parameter.
 */
 PortList getPortList ( int capabilities = 0 );

 //! Try to change the MIDI client name.
 /*! If the MIDI backend supports MIDI client names (e.g. JACK, ALSA,
   Core MIDI), this function can be used to change the name of our
   MIDI client which is presented to other applications

   \param clientName the new Name.
 */
 void setClientName ( const std::string& clientName );

  //! Try to change the MIDI port name.
  /*! If the MIDI backend supports MIDI port names (e.g. JACK, ALSA,
    Core MIDI), this function can be used to change the name of our
    MIDI port which is presented to other applications

    \param clientName the new Name.
  */
 void setPortName ( const std::string& portName );


  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    This function provides a safe interface.

    \param message The message that shall be sent

    \sa sendMessage ( const unsigned char *, size_t )
  */
  void sendMessage ( const std::vector<unsigned char>& message ) {
    sendMessage ( message.data ( ), message.size ( ) );
  }

  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    This function provides a fast interface if the date is already
    available as a byte string.

    \param message A pointer to the MIDI message as raw bytes
    \param size Length of the MIDI message in bytes
  */
  void sendMessage ( const unsigned char * message, size_t size );

  //! Specify whether certain MIDI message types should be queued or ignored during input.
  /*!
    By default, MIDI timing and active sensing messages are ignored
    during message input because of their relative high data rates.
    MIDI sysex messages are ignored by default as well. Variable
    values of "true" imply that the respective message type will be
    ignored.

    \param midiSysex true if SysEx messages are to be ignored during input
    \param midiTime  true if MIDI timing messages shall be ignored during input
    \param midiSense true fi active sensing messages shall be ignored during input
  */
  void ignoreTypes ( bool midiSysex = true,
                     bool midiTime = true,
                     bool midiSense = true );

#if 0
    //! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
  /*!
    This function returns immediately whether a new message is
    available or not. A valid message is indicated by a non-zero
    vector size. An exception is thrown if an error occurs during
    message retrieval or an input connection was not previously
    established.

    \param message a vector that will be filled with the message.
  */
  double getMessage ( std::vector<unsigned char>& message );
#endif

 //! Set a callback function to be invoked for incoming MIDI messages.
  /*!
    The callback function will be called whenever an incoming MIDI
    message is received. While not absolutely necessary, it is best
    to set the callback function before opening a MIDI port to avoid
    leaving some messages in the queue.

    \param callback A callback function must be given.
    \param userData Opitionally, a pointer to additional data can be
    passed to the callback function whenever it is called.
  */
  void setCallback ( SplitSysexMidiInterface * callback );

  //! Cancel use of the current callback function ( if one exists ) .
  /*!
    Subsequent incoming MIDI messages will be ignored until a new callback has been set.
  */
  void cancelCallback ( );


 //! Pure virtual function to set an error callback function to be invoked when an error has occured.
 /*!
   The callback function will be called whenever an error has occured. It is best
   to set the error callback function before opening a port.
 */
 void setErrorCallback ( ErrorInterface * callback );

 //! A basic error reporting function for RtMidi classes.
 void error ( const Error & e );


 protected:
 MidiApiList apis;
 bool preferSystem;
 std::string clientName;

 Midi ( bool pfsystem,
        const std::string& name );

 virtual ~Midi ( );

 MidiApi * openMidiApi ( ApiType api );
};


#undef RTMIDI_CLASSNAME

/* A deprecated type. See below for the documentation. We
   split the definiton into several pieces to work around some
   intended warnings. */
typedef void ( *ErrorCallback_t ) ( Error::Type type, const std::string& errorText, void * userdata );
//! RtMidi error callback function prototype.
/*!
  \param type Type of error.
  \param errorText Error description.

  Note that class behaviour is undefined after a critical error ( not
  a warning ) is reported.
  \sa ErrorInterface
  \deprecated
*/
RTMIDI_DEPRECATED ( typedef ErrorCallback_t ErrorCallback, "RtMidi now provides a class MidiInterface for error callbacks" );

/* A deprecated type. See below for the documentation. We
   split the definiton into several pieces to work around some
   intended warnings. */
#define ErrorCallback ErrorCallback_t

typedef void ( *MidiCallback_t ) ( double timeStamp, std::vector<unsigned char> * message, void * userData ) ;
//! C style user callback function type definition.
/*!
  This interface type has been replaced by a MidiInterface class.

  \param timeStamp timestamp indicating when the event has been received
  \param message a pointer to the binary MIDI message
  \param userData a pointer that can be set using setUserdata
  \sa MidiIn
  \sa MidiInApi
  \sa MidiInterface
  \deprecated
*/
RTMIDI_DEPRECATED ( typedef MidiCallback_t MidiCallback, "RtMidi now provides a class MidiInterface for MIDI callbacks" );
#define MidiCallback MidiCallback_t

/**********************************************************************/
/*! \class OnewayMidi
  \brief A realtime MIDI class – compatibility layer for RtMidi.

  This class provides a common, platform-independent API for realtime
  MIDI input. It allows access to a single MIDI input port. Incoming
  MIDI messages are immediately passed to a user-specified callback
  object. If no callback is set, the messages are silently
  dropped. For queued MIDI input an instance of \l MidiQueueInterface
  can be used. Create multiple instances of this class to connect to
  more than one MIDI ports at the same time. With the OS-X, Linux
  ALSA, and JACK MIDI APIs, it is also possible to open a virtual
  input port to which other MIDI software clients can connect.
*/
/**********************************************************************/

// **************************************************************** //
//
// OnewayMidi and MidiOut class declarations.
//
// OnewayMidi / MidiOut are "controllers" used to select an available
// MIDI input or output interface. They present common APIs for the
// user to call but all functionality is implemented by the classes
// OnewayMidiApi, MidiOutApi and their subclasses. OnewayMidi and MidiOut
// each create an instance of a OnewayMidiApi or MidiOutApi subclass based
// on the user's API choice. If no choice is made, they attempt to
// make a "logical" API selection.
//
// **************************************************************** //



#define RTMIDI_CLASSNAME "OnewayMidi"
struct RtMidiCompatibility: public Midi {
public:
  RtMidiCompatibility ( ApiType api=rtmidi::UNSPECIFIED,
                          unsigned int caps=rtmidi::PortDescriptor::INOUTPUT,
                          const std::string& clientName = std::string ( "RtMidi Input Client" ),
                          bool pfsystem = true )
    : Midi(api,caps,clientName,pfsystem),
      capabilities (caps) {}

  using Midi::openPort;
  //! Open a MIDI connection given by a port descriptor.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    \param port A port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.

  */
  void openPort ( const PortDescriptor& port,
                  const std::string& portName) {
    openPort (port, portName, capabilities);
  }
  void openPort ( const PortDescriptor & port ) {
    std::string name;
    switch (capabilities) {
    case PortDescriptor::INPUT:
      name = "RtMidi Input Port";
      break;
    case PortDescriptor::OUTPUT:
      name = "RtMidi Output Port";
      break;
    case PortDescriptor::INOUTPUT:
      name = "RtMidi Input and Output Port";
      break;
    default:
      name = "unknown RtMidi Port";
      break;
    }
    openPort(port,name);
  }

  //! Open a MIDI connection given by a port descriptor pointer.
 /*!
   If the port is not available for this type of connection, an exception is thrown.

   \param port A pointer to a port descriptor of the port must be specified.
   \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
   \sa openPort ( const PortDescriptor &, const std::string& );
 */
 void openPort ( Pointer<PortDescriptor> port,
                 const std::string& portName) {
    openPort (port, portName, capabilities);
  }
  void openPort ( Pointer<PortDescriptor> port) {
    std::string name;
    switch (capabilities) {
    case PortDescriptor::INPUT:
      name = "RtMidi Input Port";
      break;
    case PortDescriptor::OUTPUT:
      name = "RtMidi Output Port";
      break;
    case PortDescriptor::INOUTPUT:
      name = "RtMidi Input and Output Port";
      break;
    default:
      name = "unknown RtMidi Port";
      break;
    }
    openPort(port,name);
  }



  /* compatibility to old RtMidi */

  static constexpr const auto UNSPECIFIED = rtmidi::UNSPECIFIED;
  static constexpr const auto MACOSX_CORE = rtmidi::MACOSX_CORE;
  static constexpr const auto LINUX_ALSA = rtmidi::LINUX_ALSA;
  static constexpr const auto UNIX_JACK = rtmidi::UNIX_JACK;
  static constexpr const auto WINDOWS_MM = rtmidi::WINDOWS_MM;
  static constexpr const auto RTMIDI_DUMMY = rtmidi::DUMMY;

  RTMIDI_DEPRECATED ( typedef ApiType Api,
                      "enum RtMidi::Api has been replaced by enum rtmidi::ApiType" );

  //! Pure virtual to return the number of available MIDI ports of the current API.
  /*!
    \return This function returns the number of MIDI ports of
    the selected API.

    \note Only ports are counted that can be used with the
    current API so an input API does ignore all output devices
    and vice versa.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList instead in
    new projects.

    \deprecated

    \sa getPortList( int )
    \sa PortDascriptor
  */
  RTMIDI_DEPRECATED ( unsigned int getPortCount ( unsigned int capabilities ),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " );

  //! Pure virtual to return the number of available MIDI ports of the current API.
  /*!
    \return This function returns the number of MIDI ports of
    the selected API.

    \note Only ports are counted that can be used with the
    current API so an input API does ignore all output devices
    and vice versa.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList instead in
    new projects.

    \deprecated

    \sa getPortList( int )
    \sa PortDascriptor
  */
  RTMIDI_DEPRECATED ( unsigned int getPortCount ( ),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) {
    return getPortCount ( capabilities );
  }


  //! Pure virtual function to return a string identifier for the specified MIDI port number.
  /*!
    \param portNumber Number of the device to be referred to.
    \return The name of the port with the given Id is returned.
    \retval An empty string is returned if an invalid port specifier is provided.

    \note Only ports are counted that can be used with the
    current API so an input API does ignore all output devices
    and vice versa.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
    new projects.

    \deprecated

    \sa getPortList ( int )
    \sa getDescriptor( int )
    \sa rtmidi::PortDescriptor
  */
  RTMIDI_DEPRECATED ( std::string getPortName ( unsigned int portNumber = 0 ) ,
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " );

  using Midi::openVirtualPort;
  //! Compatibilty function for older code
  /*!
   * This function opens a virtual port. At this stage we cannot
   * reliably determine whether in which direction the data shall flow.
   *
   * use openVirtualPort( const std::string &, PortDescriptor::PortCapablilites) instead.
   *
   * \deprecated
   *
   * \sa openVirtualPort( const std::string &, PortDescriptor::PortCapablilites)
   */
  RTMIDI_DEPRECATED ( void openVirtualPort ( const std::string& portName
                                             = std::string ( "RtMidi virtual port" ) ),
                      "For better usability you should call this function from a derived class" ) {
    openVirtualPort( portName, capabilities );
  }


  //! Pure virtual function to open a MIDI connection given by enumeration number.
  /*! \param portNumber An optional port number greater than 0
    can be specified. Otherwise, the default or first port
    found is opened.

    \param portName An optional name for the applicaction port that
    will be generated to connect to portId can be specified.
    \param  capabilities Flags that indicate whether we want to open an input
    or output Port.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
    new projects.

    \deprecated

    \sa openPort( const PortDescriptor&, const std::string&, unsigned int )
    \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int )
    \sa rtmidi::PortDescriptor
  */
  RTMIDI_DEPRECATED ( void openPort ( unsigned int portNumber,
                                      const std::string& portName,
                                      unsigned int capabilities),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " );


  //! Pure virtual function to open a MIDI connection given by enumeration number.
  /*! \param portNumber An optional port number greater than 0
    can be specified. Otherwise, the default or first port
   found is opened.

   \param portName An optional name for the applicaction port that
   will be generated to connect to portId can be specified.
   \param  capabilities Flags that indicate whether we want to open an input
   or output Port.

   \warning Some APIs sort ports according to MIDI clients or
   devices. So the order of MIDI ports may change during the program
   lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
   new projects.

   \deprecated

   \sa openPort( const PortDescriptor&, const std::string&, unsigned int )
   \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int )
   \sa rtmidi::PortDescriptor
 */
 RTMIDI_DEPRECATED ( void openPort ( unsigned int portNumber,
                                     const std::string& portName = std::string ( "RtMidi port" )),
                     "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) {
   openPort( portNumber, portName, capabilities );
 }


  using Midi::sendMessage;
  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    \note This function has been replaced by sendMessage ( const std::vector<unsigned char>& )

    \deprecated

    \sa sendMessage ( const std::vector<unsigned char>& )
  */
  RTMIDI_DEPRECATED ( void sendMessage ( const std::vector<unsigned char> * message ),
                      "Please, use a C++ style reference to pass the message vector." )
  {
    if ( !message ) {
      error ( RTMIDI_ERROR ( gettext_noopt ( "No data in message argument." ),
                             Error::INVALID_PARAMETER ) );
      return;
    }
    sendMessage ( *message );
  }

  using Midi::setCallback;

  //! Set a callback function to be invoked for incoming MIDI messages.
  /*!
    The callback function will be called whenever an incoming MIDI
    message is received. While not absolutely necessary, it is best
    to set the callback function before opening a MIDI port to avoid
    leaving some messages in the queue.

    \param callback A callback function must be given.
    \param userData Opitionally, a pointer to additional data can be
    passed to the callback function whenever it is called.

    \note This function has been replaced by setCallback ( MidiInterface * ) which provides
    a more comfortable and type safe solution.

    \deprecated

    \sa setCallback ( MidiInterface * )
  */
  RTMIDI_DEPRECATED ( void setCallback ( MidiCallback callback, void * userData = 0 ),
                      "RtMidi now provides a type-safe MidiInterface class." );

  using Midi::setErrorCallback;
  //! Pure virtual function to set an error callback function to be invoked when an error has occured.
  /*!
    The callback function will be called whenever an error has occured. It is best
    to set the error callback function before opening a port.

    \note This function has been replaced by setErrorCallback ( ErrorInterface * ) which provides
    a more comfortable and type safe solution.

    \deprecated

    \sa setErrorCallback ( ErrorInterface * )
  */
  RTMIDI_DEPRECATED ( void setErrorCallback ( ErrorCallback errorCallback = NULL, void * userData = 0 ), "setErrorCallback now expects an object of type ErrorInterface" );

protected:
  unsigned int capabilities;
};
inline RTMIDI_DEPRECATED ( std::string getApiName ( RtMidiCompatibility::Api type ), "Use rtmidi::ApiType instead of RtMidi::Api" );
inline std::string getApiName ( RtMidiCompatibility::Api type )
{
  return Midi::getApiName ( ( ApiType ) type );
}
#undef RTMIDI_CLASSNAME



/**********************************************************************/
/*! \class MidiIn
  \brief A realtime MIDI input class.

  This class provides a common, platform-independent API for realtime
  MIDI input. It allows access to a single MIDI input port. Incoming
  MIDI messages are immediately passed to a user-specified callback
  object. If no callback is set, the messages are silently
  dropped. For queued MIDI input an instance of \l MidiQueueInterface
  can be used. Create multiple instances of this class to connect to
  more than one MIDI ports at the same time. With the OS-X, Linux
  ALSA, and JACK MIDI APIs, it is also possible to open a virtual
  input port to which other MIDI software clients can connect.
*/
/**********************************************************************/

// **************************************************************** //
//
// MidiIn and MidiOut class declarations.
//
// MidiIn / MidiOut are "controllers" used to select an available
// MIDI input or output interface. They present common APIs for the
// user to call but all functionality is implemented by the classes
// MidiInApi, MidiOutApi and their subclasses. MidiIn and MidiOut
// each create an instance of a MidiInApi or MidiOutApi subclass based
// on the user's API choice. If no choice is made, they attempt to
// make a "logical" API selection.
//
// **************************************************************** //

#define RTMIDI_CLASSNAME "MidiIn"
struct RTMIDI_DLL_PUBLIC MidiIn : public RtMidiCompatibility
{
 public:


  //! Default constructor that allows an optional api, client name and queue size.
  /*!
    An exception will be thrown if a MIDI system initialization
    error occurs. The queue size defines the maximum number of
    messages that can be held in the MIDI queue ( when not using a
    callback function ) . If the queue size limit is reached,
    incoming messages will be ignored.

    If no API argument is specified and multiple API support has been
    compiled, the default order of use is JACK, ALSA ( Linux ) and CORE,
    JACK ( OS-X ) .

    \param api An optional API id can be specified.
    \param clientName An optional Client name can be specified. This
    will be used to group the ports that are created
    by the application.

    \param pfsystem An optional boolean parameter can be
    provided to indicate the API preferences of the user
    code. If RtMidi is requested to autoselect a backend
    this parameter tells which backend should be tried
    first. If it is \c true the backend will prefer OS
    provieded APIs ( WinMM, ALSA, Core MIDI ) over other
    APIs ( JACK ) . If \c false, the order will be vice
    versa.
  */
  MidiIn ( ApiType api=rtmidi::UNSPECIFIED,
           const std::string& clientName = std::string ( "RtMidi Input Client" ),
           bool pfsystem = true )
    : RtMidiCompatibility( api,
            PortDescriptor::INPUT,
            clientName,
            pfsystem ) {}

  //! If a MIDI connection is still open, it will be closed by the destructor.
  virtual ~MidiIn ( void ) throw ( );
#if 0
  using Midi::openPort;


  //! Open a MIDI connection given by a port descriptor.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    \param port A port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
    \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int );
  */
  void openPort ( const PortDescriptor& port,
                  const std::string& portName = std::string ( "RtMidi" ),
                  unsigned int capabilities = PortDescriptor::INPUT ) {
    return Midi::openPort(port, portName, capabilities);
  }


  //! Open a MIDI connection given by a port descriptor pointer.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    \param port A pointer to a port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
    \sa openPort ( const PortDescriptor &, const std::string&, unsigned int );
  */
  void openPort ( Pointer<PortDescriptor> port,
                  const std::string& portName = std::string ( "RtMidi" ),
                  unsigned int capabilities = PortDescriptor::INPUT ) {
    return Midi::openPort(port, portName, capabilities);
  }

  //! Function to create a virtual port, with optional name.
  /*!
    This function creates a virtual MIDI port to which other
    software applications can connect. This type of functionality
    is currently only supported by the Macintosh OS-X, any JACK,
    and Linux ALSA APIs ( the function returns an error for the other APIs ) .

    If the port is not available for this type of connection, an exception is thrown.

    \param portName An optional name for the application port that is
    used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
  */
  void openVirtualPort ( const std::string& portName = std::string ( "RtMidi virtual input port" ),
                         unsigned int capabilities = PortDescriptor::INPUT ) {
    return Midi::openVirtualPort(portName, capabilities);
  }

#if 0
  //! Function to create a virtual port, with optional name.
  /*!
    This function creates a virtual MIDI port to which other
    software applications can connect. This type of functionality
    is currently only supported by the Macintosh OS-X, any JACK,
    and Linux ALSA APIs ( the function returns an error for the other APIs ) .

    If the port is not available for this type of connection, an exception is thrown.

    \param portName An optional name for the application port that is
    used to connect to portId can be specified.
  */
  void openVirtualPort ( const std::string& portName = std::string ( "RtMidi virtual input port" )) {
    return Midi::openVirtualPort(portName, PortDescriptor::INPUT);
  }
#endif

  //! Return a list of all available ports of the current API.
  /*!
    \param capabilities an optional parameter that
    describes which capabilities the returned devices
    must support. The returned devices may have
    additional capabilities to those which have been
    requested, but not less.

    \return This function returns a list of port descriptors.

    \note Each API will request additonal
    capabilites. An output API will set always add \ref
    PortDescriptor::OUTPUT to the mask while an input
    device will always add \ref PortDescriptor::OUTPUT.

    \note An input API may but need not necessarily
    report output devices which cannot be used as input
    if \ref PortDescriptor::OUTPUT is passed as \ref
    capabilities parameter.
  */
  PortList getPortList ( int capabilities = PortDescriptor::INPUT ) {
    return Midi::getPortList ( capabilities );
  }


  //! Pure virtual function to open a MIDI connection given by enumeration number.
  /*! \param portNumber An optional port number greater than 0
    can be specified. Otherwise, the default or first port
    found is opened.

    \param portName An optional name for the applicaction port that
    will be generated to connect to portId can be specified.
    \param  capabilities Flags that indicate whether we want to open an input
    or output Port.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
    new projects.

    \deprecated

    \sa openPort( const PortDescriptor&, const std::string&, unsigned int )
    \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int )
    \sa rtmidi::PortDescriptor
  */
  RTMIDI_DEPRECATED ( void openPort ( unsigned int portNumber,
                                      const std::string& portName = "RtMidi Input Port"),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) {
    openPort( portNumber, portName, PortDescriptor::INPUT );
  }
#endif
};
#undef RTMIDI_CLASSNAME

/**********************************************************************/
/*! \class MidiOut
  \brief A realtime MIDI output class.

  This class provides a common, platform-independent API for MIDI
  output. It is a wrapper class around rtmidi::Midi which predefines
  the optional capabilites argument to
  rtmidi::PortDescriptor::OUTPUT. It allows one to probe available
  MIDI output ports, to connect to one such port, and to send MIDI
  bytes immediately over the connection. Create multiple instances of
  this class to connect to more than one MIDI device at the same
  time. With the OS-X, Linux ALSA and JACK MIDI APIs, it is also
  possible to open a virtual port to which other MIDI software clients
  can connect.


*/
/**********************************************************************/

#define RTMIDI_CLASSNAME "MidiOut"
struct RTMIDI_DLL_PUBLIC MidiOut : public RtMidiCompatibility
{
 public:

  //! Default constructor that allows an optional client name.
  /*!
    An exception will be thrown if a MIDI system initialization error occurs.

    If no API argument is specified and multiple API support has been
    compiled, the default order of use is JACK, ALSA ( Linux ) and CORE,
    JACK ( OS-X ) .

    \param api An optional API id can be specified.
    \param clientName An optional Client name can be specified. This
    will be used to group the ports that are created
    by the application.

    \param pfsystem An optional boolean parameter can be
    provided to indicate the API preferences of the user
    code. If RtMidi is requested to autoselect a backend
    this parameter tells which backend should be tried
    first. If it is \c true the backend will prefer OS
    provieded APIs ( WinMM, ALSA, Core MIDI ) over other
    APIs ( JACK ) . If \c false, the order will be vice
    versa.
  */
  MidiOut ( ApiType api=rtmidi::UNSPECIFIED,
            const std::string& clientName = std::string ( "RtMidi Output Client" ),
            bool pfsystem = true )
    : RtMidiCompatibility( api, PortDescriptor::OUTPUT, clientName, pfsystem ) {}

  //! The destructor closes any open MIDI connections.
  virtual ~MidiOut ( void ) throw ( );
#if 0
  using Midi::openPort;

  //! Open a MIDI connection given by a port descriptor.
  /*!
    \param port A port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
  */
  void openPort ( const PortDescriptor& port,
                  const std::string& portName = std::string ( "RtMidi" ),
                  unsigned int capabilities = PortDescriptor::OUTPUT ) {
    Midi::openPort( port, portName, capabilities);
  }

  //! Open a MIDI connection given by a port descriptor pointer.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    \param port A pointer to a port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
    \sa openPort ( const PortDescriptor& , const std::string& );
  */
  void openPort ( Pointer<PortDescriptor> port,
                  const std::string& portName = std::string ( "RtMidi" ),
                  unsigned int capabilities = PortDescriptor::OUTPUT ) {
    Midi::openPort( port, portName, capabilities);
  }

  //! Function to create a virtual port, with optional name.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    This function creates a virtual MIDI port to which other
    software applications can connect. This type of functionality
    is currently only supported by the Macintosh OS-X, any JACK,
    and Linux ALSA APIs ( the function returns an error for the other APIs ) .

    \param portName An optional name for the applicaction port that is
    used to connect to portId can be specified.
    \param capabilities The kind of connection we shall establish.
  */
  void openVirtualPort ( const std::string& portName = std::string ( "RtMidi virtual output port" ),
                         unsigned int capabilities  =  PortDescriptor::OUTPUT ) {
    Midi::openVirtualPort( portName, capabilities);
  };

  //! Return a list of all available ports of the current API.
  /*!
    If the port is not available for this type of connection, an exception is thrown.

    \param capabilities an optional parameter that
    describes which capabilities the returned devices
    must support. The returned devices may have
    additional capabilities to those which have been
    requested, but not less.

    \return This function returns a list of port descriptors.

    \note Each API will request additonal
    capabilites. An output API will set always add \ref
    PortDescriptor::OUTPUT to the mask while an input
    device will always add \ref PortDescriptor::OUTPUT.

    \note An input API may but need not necessarily
    report output devices which cannot be used as input
    if \ref PortDescriptor::OUTPUT is passed as \ref
    capabilities parameter.
  */
  PortList getPortList ( int capabilities = PortDescriptor::OUTPUT ) {
    return Midi::getPortList ( capabilities );
  }

  //! Pure virtual function to open a MIDI connection given by enumeration number.
  /*! \param portNumber An optional port number greater than 0
    can be specified. Otherwise, the default or first port
    found is opened.

    \param portName An optional name for the applicaction port that
    will be generated to connect to portId can be specified.
    \param  capabilities Flags that indicate whether we want to open an input
    or output Port.

    \warning Some APIs sort ports according to MIDI clients or
    devices. So the order of MIDI ports may change during the program
    lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
    new projects.

    \deprecated

    \sa openPort( const PortDescriptor&, const std::string&, unsigned int )
    \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int )
    \sa rtmidi::PortDescriptor
  */
  RTMIDI_DEPRECATED ( void openPort ( unsigned int portNumber,
                                      const std::string& portName = "RtMidi Output Port"),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) {
    openPort (portNumber,portName, PortDescriptor::OUTPUT);
  }
#endif
};
#undef RTMIDI_CLASSNAME


// **************************************************************** //
//
// MidiInApi / MidiOutApi class declarations.
//
// Subclasses of MidiInApi and MidiOutApi contain all API- and
// OS-specific code necessary to fully implement the RtMidi API.
//
// Note that MidiInApi and MidiOutApi are abstract base classes and
// cannot be explicitly instantiated. MidiIn and MidiOut will
// create instances of a MidiInApi or MidiOutApi subclass.
//
// **************************************************************** //

#define RTMIDI_CLASSNAME "MidiApi"
class RTMIDI_DLL_PUBLIC MidiApi
  {
  public:

  MidiApi ( );
  virtual ~MidiApi ( );

  //! Returns the MIDI API specifier for the current instance of RtMidiIn.
  virtual ApiType getCurrentApi ( void ) throw ( ) = 0;

  //! Return the capabilities the API has in general.
  /**
   * This functions can be used to query the API about the
   * capabilities it can have.
   *
   * \return A bit mask of the constants from unsigned int.
   */
  virtual unsigned int getCapabilities ( ) throw ( ) = 0;

  //! Return whether the API supports virtual ports
  /*!
    \retval true The funcion returns true if the API supports virtual ports.
    \retval false The funcion returns false if the API doesn't support virtual ports.
    \sa openVirtualPort
  */
  virtual bool hasVirtualPorts ( ) const = 0;

  //! Returns the largest number of bytes a system exclusive message can have.
  /*!  This function returns the maximum number of bytes the message
    can theoretically have. Practically the maximum size may be
    smaller if the output buffer is not empty.

    \return The number of bytes a system exclusive message can hae.
   */
  virtual size_t maxSysExSize ( ) const = 0;


  // ! A basic error reporting function for RtMidi classes.
  // static void error ( Error::Type type, std::string& errorString );

  //! Pure virtual function to return whether a port is open or not.
  /*! \retval true if a port is open and
    \retval false if the port is not open ( e.g. not opend or closed ) .
  */
  bool isPortOpen ( ) const { return connected_; }

  //! Pure virtal function to create a virtual port, with optional name.
  /*!
    This function creates a virtual MIDI port to which other
    software applications can connect. This type of functionality
    is currently only supported by the Macintosh OS-X, any JACK,
    and Linux ALSA APIs ( the function returns an error for the other APIs ) .

    \param portName An optional name for the applicaction port that is
    used to connect to portId can be specified.

    \sa hasVirtualPorts
  */
  virtual void openVirtualPort ( const std::string& portName,
                                 unsigned int capabilities) = 0;

  //! Pure virtual function to open a MIDI connection given by a port descriptor.
  /*!
    \param port A port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
  */
  virtual void openPort ( const PortDescriptor& port,
                          const std::string& portName,
                          unsigned int capabilities ) = 0;


  //! Open a MIDI connection given by a port descriptor pointer.
  /*!
    \param port A pointer to a port descriptor of the port must be specified.
    \param portName An optional name for the applicaction port that is used to connect to portId can be specified.
    \sa openPort ( const PortDescriptor& , const std::string& );
  */
  void openPort ( Pointer<PortDescriptor> port,
                  const std::string& portName,
                  unsigned int capabilities ) {
    if ( !port ) {
      error ( RTMIDI_ERROR ( gettext_noopt ( "Passed NULL pointer." ),
                             Error::INVALID_PARAMETER ) );
      return;
    }
    openPort ( *port, portName, capabilities );
  }

  //! Pure virtual function to close an open MIDI connection ( if one exists ) .
  virtual void closePort ( void ) = 0;

  //! Pure virtual function to return a port descriptor if the port is open
  /*! This function returns a port descriptor that can be used to open another port
    either to the connected port or – if the backend supports it – the connecting port.
    \param local The parameter local defines whether the function returns a descriptor to
    the virtual port ( true ) or the remote port ( false ) . The function returns 0 if the
    port cannot be determined ( e.g. if the port is not connected or the backend dosen't support it ) .
  */
  virtual Pointer<PortDescriptor> getDescriptor ( bool local=false ) = 0;

  //! Pure virtual function to return a list of all available ports of the current API.
  /*!
    \param capabilities an optional parameter that describes which
    capabilities the device typu

    \return This function returns a list of port descriptors.

    \note An input API may but need not necessarily report
    output devices which cannot be used as input if
    \ref 0 is passed as \ref capabilities parameter.
    \sa PortDescriptor::PortCapabilitiers
  */
  virtual PortList getPortList ( int capabilities = 0 ) = 0;

  //! Try to change the MIDI client name.
  /*! If the MIDI backend supports MIDI client names (e.g. JACK, ALSA,
    Core MIDI), this function can be used to change the name of our
    MIDI client which is presented to other applications

    \param clientName the new Name.
  */
  virtual void setClientName ( const std::string& clientName ) = 0;

  //! Try to change the MIDI port name.
  /*! If the MIDI backend supports MIDI port names (e.g. JACK, ALSA,
    Core MIDI), this function can be used to change the name of our
    MIDI port which is presented to other applications

    \param clientName the new Name.
  */
  virtual void setPortName ( const std::string& portName ) = 0;

  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    This function provides a safe interface.

    \param message The message that shall be sent

    \sa sendMessage ( const unsigned char *, size_t )
  */
  void sendMessage ( const std::vector<unsigned char>& message )
  {
    if ( message.empty ( ) ) {
      error ( RTMIDI_ERROR ( gettext_noopt ( "No data in message argument." ),
                             Error::WARNING ) );
    }
    sendMessage ( message.data ( ), message.size ( ) );
  }

  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    This function provides a fast interface if the date is already
    available as a byte string.

    \param message A pointer to the MIDI message as raw bytes
    \param size Length of the MIDI message in bytes
  */
  virtual void sendMessage ( const unsigned char * message, size_t size ) = 0;

  //! Specify whether certain MIDI message types should be queued or ignored during input.
  /*!
    By default, MIDI timing and active sensing messages are ignored
    during message input because of their relative high data rates.
    MIDI sysex messages are ignored by default as well. Variable
    values of "true" imply that the respective message type will be
    ignored.

    \param midiSysex true if SysEx messages are to be ignored during input
    \param midiTime  true if MIDI timing messages shall be ignored during input
    \param midiSense true fi active sensing messages shall be ignored during input
  */
  virtual void ignoreTypes ( bool midiSysex, bool midiTime, bool midiSense ) = 0;

#if 0
    //! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
  /*!
    This function returns immediately whether a new message is
    available or not. A valid message is indicated by a non-zero
    vector size. An exception is thrown if an error occurs during
    message retrieval or an input connection was not previously
    established.

    \param message a vector that will be filled with the message.
  */
  double getMessage ( std::vector<unsigned char>& message );
#endif

  //! Set a callback function to be invoked for incoming MIDI messages.
  /*!
    The callback function will be called whenever an incoming MIDI
    message is received. While not absolutely necessary, it is best
    to set the callback function before opening a MIDI port to avoid
    leaving some messages in the queue.

    \param callback A callback function must be given.
    \param userData Opitionally, a pointer to additional data can be
    passed to the callback function whenever it is called.
  */
  virtual void setCallback ( SplitSysexMidiInterface * callback ) = 0;

  //! Cancel use of the current callback function ( if one exists ) .
  /*!
    Subsequent incoming MIDI messages will be written to the queue
    and can be retrieved with the \e getMessage function.
  */
  virtual void cancelCallback ( void ) = 0;

  //! Virtual function to set the error callback object
  /*!
    Everytime an error is detected or a warning is issued the function
    \r ErrorInterface::rtmidi_error of the callback object is called with an error object
    that describes the situation.

    \param callback An objact that provides an ErrorInterface.
  */
  virtual void setErrorCallback ( ErrorInterface * callback );


  //! A basic error reporting function for RtMidi classes.
  /*! This function hanles errors end warnings that
    occur during runtime of RtMidi. If an error callback
    has been set the function calls the callback and
    returns quietly assuming the callback handled the
    case correctly.

    Otherwise it depends on the type of the error. If it
    is a warning, a message is displayed to
    std::cerr. If it is an error the object is thrown as
    an exception.

    \param e Error/Warning object describing the current
    situation.

    \throw Error
  */
  void error ( const Error & e );

  // deprecated functions

    //! Pure virtual to return the number of available MIDI ports of the current API.
  /*!
    \return This function returns the number of MIDI ports of
    the selected API.

    \note Only ports are counted that can be used with the
    current API so an input API does ignore all output devices
    and vice versa.


   \warning Some APIs sort ports according to MIDI clients or
   devices. So the order of MIDI ports may change during the program
   lifetime. It is highly recommended to use getPortList instead in
   new projects.

   \deprecated

   \sa getPortList( int )
   \sa PortDascriptor
  */
  virtual
  RTMIDI_DEPRECATED ( unsigned int getPortCount ( unsigned int capabilities ),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) {
    return getPortList( capabilities ).size();
  }

  //! Pure virtual function to return a string identifier for the specified MIDI port number.
  /*!
    \param portNumber Number of the device to be referred to.
    \return The name of the port with the given Id is returned.
    \retval An empty string is returned if an invalid port specifier is provided.

    \note Only ports are counted that can be used with the
    current API so an input API does ignore all output devices
    and vice versa.


   \warning Some APIs sort ports according to MIDI clients or
   devices. So the order of MIDI ports may change during the program
   lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
   new projects.

   \deprecated

   \sa getPortList ( int )
   \sa getDescriptor( int )
   \sa rtmidi::PortDescriptor
  */
  virtual
  RTMIDI_DEPRECATED ( std::string getPortName ( unsigned int portNumber,
                                                unsigned int capabilities ),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) " ) = 0;


#if 0
  //! Compatibilty function for older code
  /*!
  * This function opens a virtual port. At this stage we cannot
  * reliably determine whether in which direction the data shall flow.
  *
  * use openVirtualPort( const std::string &, PortDescriptor::PortCapablilites) instead.
  *
  * \deprecated
  *
  * \sa openVirtualPort( const std::string &, PortDescriptor::PortCapablilites)
  */
  virtual
  RTMIDI_DEPRECATED ( void openVirtualPort ( const std::string& portName
                                             = std::string ( "RtMidi virtual port" ) ),
                      "For better usability you should call this function from a derived class or provide a secound argument for the port capablilities" ) {
    openVirtualPort( portName, currentCapabilities );
  }
#endif

  //! Pure virtual function to open a MIDI connection given by enumeration number.
  /*! \param portNumber An optional port number greater than 0
    can be specified. Otherwise, the default or first port
    found is opened.

    \param portName An optional name for the applicaction port
    that will be generated to connect to portId can be
    specified.

   \warning Some APIs sort ports according to MIDI clients or
   devices. So the order of MIDI ports may change during the program
   lifetime. It is highly recommended to use getPortList( int ) or getDescriptor( int ) instead in
   new projects.

   \deprecated

   \sa openPort( const PortDescriptor&, const std::string&, unsigned int )
   \sa openPort ( Pointer<PortDescriptor>, const std::string&, unsigned int )
   \sa rtmidi::PortDescriptor
  */
  virtual
  RTMIDI_DEPRECATED ( void openPort ( unsigned int portNumber,
                                      const std::string& portName,
                                      unsigned int capabilities ),
                      "Port numbers are unreliable. Use port descriptors instead ( see examples for a demonstration ) ") = 0;

  //! Immediately send a single message out an open MIDI output port.
  /*!
    An exception is thrown if an error occurs during output or an
    output connection was not previously established.

    \note This function has been replaced by sendMessage ( const std::vector<unsigned char>& )

    \deprecated

    \sa sendMessage ( const std::vector<unsigned char>& )
  */
  RTMIDI_DEPRECATED ( void sendMessage ( const std::vector<unsigned char> * message ),
                      "Please, use a C++ style reference to pass the message vector." )
    {
      if ( !message ) {
        error ( RTMIDI_ERROR ( gettext_noopt ( "No data in message argument." ),
                               Error::INVALID_PARAMETER ) );
        return;
      }
      sendMessage ( * message );
    }




 //! Set a callback function to be invoked for incoming MIDI messages.
 /*!
   The callback function will be called whenever an incoming MIDI
   message is received. While not absolutely necessary, it is best
   to set the callback function before opening a MIDI port to avoid
   leaving some messages in the queue.

   \param callback A callback function must be given.
   \param userData Opitionally, a pointer to additional data can be
   passed to the callback function whenever it is called.

   \note This function has been replaced by setCallback ( MidiInterface * ) which provides
   a more comfortable and type safe solution.

   \deprecated

   \sa setCallback ( MidiInterface * )
 */
  RTMIDI_DEPRECATED ( void setCallback ( MidiCallback callback, void * userData = 0 ),
                      "RtMidi now provides a type-safe MidiInterface class." );

  //! Virtual function to set an error callback function to be invoked when an error has occured.
  /*!
    The callback function will be called whenever an error has occured. It is best
    to set the error callback function before opening a port.

   \note This function has been replaced by setErrorCallback ( ErrorInterface * ) which provides
   a more comfortable and type safe solution.

   \deprecated

   \sa setErrorCallback ( ErrorInterface * )
*/
  RTMIDI_DEPRECATED ( virtual void setErrorCallback ( ErrorCallback errorCallback = NULL, void * userData = 0 ), "RtMidi now provides a typesafe ErrorInterface class" );




  protected:
  bool connected_;
  bool firstErrorOccurred_;
  std::string errorString_;
  ErrorInterface * errorCallback_;
#if 0
  unsigned int currentCapabilities ;

  // These are internals shouldn't be exposed to the public API

  // The RtMidiInData structure is used to pass private class data to
  // the MIDI input handling function or thread.
  bool firstMessage;
  bool continueSysex;
  MidiMessage message;
  unsigned char ignoreFlags;
  std::atomic_bool doInput;
  friend struct JackBackendCallbacks;
#endif
  };
#undef RTMIDI_CLASSNAME

// **************************************************************** //
//
// Inline rtmidi::PortDescriptor definitions.
//
// **************************************************************** //
// rtmidi::PortDescriptor
#define RTMIDI_CLASSNAME "PortDescriptor"
inline MidiInApi * PortDescriptor :: getInputApi (  ) const
{
  return dynamic_cast<MidiInApi *>(getApi(PortDescriptor::INPUT));
}
inline MidiOutApi * PortDescriptor :: getOutputApi ( ) const
{
  return dynamic_cast<MidiOutApi *>(getApi(PortDescriptor::OUTPUT));
}
#undef RTMIDI_CLASSNAME



// **************************************************************** //
//
// Inline rtmidi::Midi, rtmidi::MidiIn and rtmidid:MidiOut definitions.
//
// **************************************************************** //
// rtmidi::Midi
#define RTMIDI_CLASSNAME "Midi"
inline std::vector<ApiType> Midi :: getCompiledApi ( bool preferSystem ) throw ( ) {
  std::vector<ApiType> apis;
  getCompiledApi ( apis, preferSystem );
  return apis;
}

inline ApiType Midi :: getCurrentApi ( void ) throw ( ) {
  if ( !apis.empty() ) return apis.front()->getCurrentApi ( );
  else return rtmidi::UNSPECIFIED;
}

inline Pointer<PortDescriptor> Midi :: getDescriptor ( bool local ) {
  if ( !apis.empty() ) return apis.front()->getDescriptor ( local );
  return 0;
}

inline PortList Midi :: getPortList ( int capabilities ) {
  if ( !apis.empty ( ) ) {
    PortList retval;
    for ( MidiApiList::iterator i = apis.begin ( );
          i != apis.end ( );
          ++i ) {
      PortList tmp = ( *i ) ->getPortList ( capabilities );
      retval.splice ( retval.end ( ), tmp );
    }
    return retval;
  }
  return PortList ( );
}

inline bool Midi :: hasVirtualPorts ( ) {
  if ( !apis.empty ( ) ) {
    for ( auto & api : apis ) {
      if ( api->hasVirtualPorts ( ) ) {
        return true;
      }
    }
  }
  return false;
}

inline size_t Midi :: maxSysExSize ( ) const {
  size_t retval = 0;
  if ( !apis.empty ( ) ) {
    for ( auto & api : apis ) {
      size_t apiSysExSize = api->maxSysExSize ( );
      retval = retval?(apiSysExSize?std::min(retval,apiSysExSize):retval):apiSysExSize;
    }
    return retval;
  }
  return 0;
}


inline void Midi :: closePort ( void ) {
  if (!apis.empty())
    apis.front()->closePort ( );
}

inline void Midi :: setClientName ( const std::string& clientName )
{
  if (!apis.empty())
    apis.front()->setClientName ( clientName );
}

inline void Midi :: setPortName ( const std::string& portName )
{
  if (!apis.empty())
    apis.front()->setPortName ( portName );
}

inline bool Midi :: isPortOpen ( void ) const {
  if (!apis.empty())
    return apis.front()->isPortOpen ( );
  return false;
}

inline void Midi :: setErrorCallback ( ErrorInterface * callback ) {
  if (!apis.empty())
    apis.front()->setErrorCallback ( callback );
}

#if 0
inline void Midi :: getCompiledApi ( std::vector<Api>& apis, bool
                                     preferSystem ) throw ( ) {
  std::vector<rtmidi::ApiType> api2;
  Midi::getCompiledApi ( api2, preferSystem );
  apis.reserve ( api2.size ( ) );
  size_t s = api2.size ( );
  for ( size_t i = 0; i < s; i++ ) {
    apis.push_back ( ( Api ) api2[i] );
  }
}
#endif

inline Midi :: Midi ( bool pfsystem,
                      const std::string& name )
  : apis ( ),
    preferSystem ( pfsystem ),
    clientName ( name ) { }

inline Midi :: ~Midi ( ) {
}

inline void Midi :: openPort ( const PortDescriptor& port,
                               const std::string& portName,
                               unsigned int capabilities) {
  if ( apis.empty()) {
    MidiApi * api = port.getApi ( capabilities );
    if (api) apis.emplace_front(api);
  }
  if (!apis.empty())
    apis.front()->openPort ( port, portName, capabilities );
}

inline void Midi :: openPort ( Pointer<PortDescriptor> port,
                               const std::string& portName,
                               unsigned int capabilities ) {
  if ( !port ) {
    error ( RTMIDI_ERROR ( gettext_noopt ( "A NULL pointer has been passed as port descriptor" ),
                           Error::INVALID_PARAMETER ) );
    return;
  }
  openPort ( *port, portName, capabilities );
}
inline void Midi :: openVirtualPort ( const std::string& portName,
                                      unsigned int capabilities ) {
  for (auto & api : apis) {
    if (api->hasVirtualPorts()) {
      api->openVirtualPort ( portName, capabilities );
      MidiApiList newlist(1, std::move(api));
      apis.swap(newlist);
      return;
    }
  }
  error ( RTMIDI_ERROR ( gettext_noopt ( "No valid MIDI system has been selected." ),
                         Error::INVALID_DEVICE ) );
}
inline void Midi :: setCallback ( SplitSysexMidiInterface * callback ) {
 if (!apis.empty())
    apis.front()->setCallback ( callback );
}
inline void Midi :: cancelCallback ( ) {
 if (!apis.empty())
    apis.front()->cancelCallback ( );
}
inline void Midi :: ignoreTypes ( bool midiSysex,
                                  bool midiTime,
                                  bool midiSense ) {
  if (!apis.empty())
    apis.front()->ignoreTypes ( midiSysex, midiTime, midiSense );
}
inline void Midi :: sendMessage ( const unsigned char * message, size_t size ) {
  if ( !message ) {
    error ( RTMIDI_ERROR ( gettext_noopt ( "No data in MIDI message." ),
                           Error::INVALID_PARAMETER ) );
    return;
  }
 if (!apis.empty())
    apis.front()->sendMessage ( message, size );
  else
    error ( RTMIDI_ERROR ( gettext_noopt ( "No valid MIDI system has been selected." ),
                           Error::WARNING ) );
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "RtMidiCompatibility"
inline void RtMidiCompatibility :: openPort ( unsigned int portNumber,
                               const std::string& portName,
                               unsigned int capabilities) {
  if (!apis.empty())
    apis.front()->openPort ( portNumber, portName, capabilities );
}
inline unsigned int RtMidiCompatibility :: getPortCount ( unsigned int capabilities ) {
  if (!apis.empty())
    return apis.front()->getPortCount ( capabilities );
  return 0;
}
inline std::string RtMidiCompatibility :: getPortName ( unsigned int portNumber ) {
  if (!apis.empty())
    return apis.front()->getPortName ( portNumber, capabilities);
  return "";
}
#if 0
inline void RtMidiCompatibility :: openVirtualPort ( const std::string& portName ) {
  if (!apis.empty())
    apis.front()->openVirtualPort ( portName );
}
#endif

inline void RtMidiCompatibility :: setErrorCallback ( ErrorCallback errorCallback , void * userData ) {
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    if (!apis.empty())
    return apis.front()->setErrorCallback ( errorCallback, userData );
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}
inline void RtMidiCompatibility :: setCallback ( MidiCallback callback, void * userData ) {
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
 if (!apis.empty())
    apis.front()->setCallback ( callback, userData );
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}
#undef RTMIDI_CLASSNAME

// old API


//! Compatibility interface to hold a C style callback function
struct CompatibilityMidiInterface: MidiInterface {
  CompatibilityMidiInterface ( MidiCallback cb, void * ud )
    : callback ( cb ),
      userData ( ud ) {}
  void midiIn ( double timestamp, std::vector<unsigned char>& message ) throw() {
    callback ( timestamp, &message, userData );
  }
  void deleteMe ( ) { delete this; }
  MidiCallback callback;
  void * userData;
};


struct CompatibilityErrorInterface: ErrorInterface {
  CompatibilityErrorInterface ( ErrorCallback cb, void * ud )
    : callback ( cb ),
      userdata ( ud ) {}
  void rtmidi_error ( const Error & e ) {
    callback ( e.getType ( ), e.getMessage ( ), userdata );
  }

  void deleteMe ( ) { delete this; }
private:
  ErrorCallback callback;
  void * userdata;
};


#define RTMIDI_CLASSNAME "MidiApi"
inline void MidiApi :: setCallback ( MidiCallback callback, void * userData )
{
  if ( !callback ) {
    error( RTMIDI_ERROR( gettext_noopt( "The callback function value is invalid." ),
                         Error::WARNING ) );
    return;
  }

  auto userCallback = new CompatibilityMidiInterface( callback, userData );
  if (userCallback) setCallback(userCallback);
}
#undef RTMIDI_CLASSNAME

RTMIDI_NAMESPACE_END

typedef rtmidi::RtMidiCompatibility RTMIDI_DEPRECATED ( RtMidi, "RtMidi has been replaced by rtmidi::Midi" );

#define RTMIDI_CLASSNAME "RtMidiIn"
struct RtMidiIn: public rtmidi::RtMidiCompatibility {
public:
  RTMIDI_DEPRECATED ( RtMidiIn ( RtMidi::Api api = rtmidi::UNSPECIFIED,
                                 const std::string& clientName = std::string ( "RtMidi Input Client" ),
                                 unsigned int queueSizeLimit = 100),
                      "Class RtMidiIn has been replaced by rtmidi::MidiIn" )
    : RtMidiCompatibility ( api, rtmidi::PortDescriptor::INPUT, clientName ), queue(queueSizeLimit) {
    setCallback(&queue);
  }

  ~RtMidiIn() {
    RtMidiCompatibility::cancelCallback();
  }

  void setCallback ( rtmidi::SplitSysexMidiInterface * callback ) {
    RtMidiCompatibility::setCallback(callback?callback:&queue);
  }

  RTMIDI_DEPRECATED ( void setCallback ( rtmidi::MidiCallback callback, void * userData = 0 ),
                      "RtMidi now provides a type-safe RtMidiCompatibilityterface class." ) {
    if (callback)
      RtMidiCompatibility::setCallback(callback, userData);
    else
      RtMidiCompatibility::setCallback(&queue);
  }

  //! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
  /*!
    This function returns immediately whether a new message is
    available or not. A valid message is indicated by a non-zero
    vector size. An exception is thrown if an error occurs during
    message retrieval or an input connection was not previously
    established.

    \note This function has been replaced by getMessage ( const std::vector<unsigned char>& )

    \deprecated

    \sa getMessage ( const std::vector<unsigned char>& )
  */
  RTMIDI_DEPRECATED ( double getMessage ( std::vector<unsigned char> * message ),
                      "Please, use a C++ style reference to pass the message vector." )
  {
    if ( !message ) {
      error ( RTMIDI_ERROR ( gettext_noopt ( "Passed NULL pointer." ),
                             rtmidi::Error::INVALID_PARAMETER ) );
      return -100.0;
    }
    return getMessage ( *message );
  }

  //! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
  /*!
    This function returns immediately whether a new message is
    available or not. A valid message is indicated by a non-zero
    vector size. An exception is thrown if an error occurs during
    message retrieval or an input connection was not previously
    established.

    \param message a vector that will be filled with the message.
  */
  double getMessage ( std::vector<unsigned char>& message ) {
    return queue.getMessage(message);
  }

  void cancelCallback ( ) {
    RtMidiCompatibility::setCallback(&queue);
  }

protected:
  rtmidi::MidiQueueInterface queue;
};
#undef RTMIDI_CLASSNAME



#define RTMIDI_CLASSNAME "RtMidiIn"

class RtMidiOut: public rtmidi::RtMidiCompatibility {
public:
  RTMIDI_DEPRECATED ( RtMidiOut ( RtMidi::Api api = rtmidi::UNSPECIFIED,
                                  const std::string& clientName = std::string ( "RtMidi Output Client" ) ),
                      "Class RtMidiOut has been replaced by rtmidi::MidiOut" )
    : RtMidiCompatibility ( ( rtmidi::ApiType ) api,
                            rtmidi::PortDescriptor::OUTPUT,
                            clientName ) {}
};
#undef RTMIDI_CLASSNAME

typedef rtmidi::Error RtMidiError;


#endif

