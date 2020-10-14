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

#include "src/ALSA.h"
#include <sstream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <functional>
#include <cerrno>

#include <pthread.h>
#include <sys/time.h>

// ALSA header file.
#include <alsa/asoundlib.h>
#ifndef RTMIDI_FALLTHROUGH
#define RTMIDI_FALLTHROUGH
#endif


//*********************************************************************//
// API: LINUX ALSA SEQUENCER
//*********************************************************************//

// API information found at:
// - http://www.alsa-project.org/documentation.php#Library

#if defined( __LINUX_ALSA__ )


RTMIDI_NAMESPACE_START
// The ALSA Sequencer API is based on the use of a callback function for
// MIDI input.
//
// Thanks to Pedro Lopez-Cabanillas for help with the ALSA sequencer
// time stamps and other assorted fixes!!!

// If you don't need timestamping for incoming MIDI events, define the
// preprocessor definition AVOID_TIMESTAMPING to save resources
// associated with the ALSA sequencer queues.

inline unsigned int capabilities2ALSA ( unsigned int capabilities ) {
  unsigned int result =
    ( ( capabilities & PortDescriptor::INPUT ) ? SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE : 0 );
  if ( capabilities & PortDescriptor::OUTPUT )
    result |= SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ;
  return result;
}




//#define PORT_TYPE( pinfo, bits ) ( ( snd_seq_port_info_get_capability( pinfo ) & ( bits ) ) == ( bits ) )

//*********************************************************************//
// API: LINUX ALSA
// Class Definitions: MidiInAlsa
//*********************************************************************//

#define RTMIDI_CLASSNAME "alsaSequencer"

template<int locking>
template<class E>
void AlsaSequencer<locking>::error(const E & e) throw() {
  std::unique_lock<portlist_t> lock(ports);
  error(e,lock);
}

template<int locking>
template<class E>
void AlsaSequencer<locking>::error(const E & e, std::unique_lock<portlist_t> &lock) throw() {
  if ( lock.mutex() != &ports || !lock.owns_lock() ) {
    error(e);
    return;
  }
  for (size_t i = 0; i <ports.size(); ++i) {
    MidiAlsa * p = dynamic_cast<MidiAlsa *>(ports[i]);
    if (p) {
      try {
        p->error(e);
      } catch (...) {}
      break;
    }
  }
}


struct alsaevententry {
  char * name;
};


const char * alsatypenames[] =
  {
   [SND_SEQ_EVENT_SYSTEM] = "System",
   [SND_SEQ_EVENT_RESULT] = "Result",
   [2] = "2",
   [3] = "3",
   [4] = "4",
   [SND_SEQ_EVENT_NOTE] = "Note",
   [SND_SEQ_EVENT_NOTEON] = "Note on",
   [SND_SEQ_EVENT_NOTEOFF] = "Note off",
   [SND_SEQ_EVENT_KEYPRESS] = "Keypress",
   [9] = "9",
   [SND_SEQ_EVENT_CONTROLLER] = "Controller",
   [SND_SEQ_EVENT_PGMCHANGE] = "Program Change",
   [SND_SEQ_EVENT_CHANPRESS] = "Channel Pressure",
   [SND_SEQ_EVENT_PITCHBEND] = "Pitch Bend",
   [SND_SEQ_EVENT_CONTROL14] = "Control 14",
   [SND_SEQ_EVENT_NONREGPARAM] = "NRPC",
   [SND_SEQ_EVENT_REGPARAM] = "RPC",
   [17] = "17",
   [18] = "18",
   [19] = "19",
   [SND_SEQ_EVENT_SONGPOS] = "Song Position",
   [SND_SEQ_EVENT_SONGSEL] = "Song select",
   [SND_SEQ_EVENT_QFRAME] = "Quarter Frame",
   [SND_SEQ_EVENT_TIMESIGN] = "Time Signature",
   [SND_SEQ_EVENT_KEYSIGN] = "Key Signature",
   [25] = "25",
   [26] = "26",
   [27] = "27",
   [28] = "28",
   [29] = "29",
   [SND_SEQ_EVENT_START] = "Start",
   [SND_SEQ_EVENT_CONTINUE] = "Continue",
   [SND_SEQ_EVENT_STOP] = "Stop",
   [SND_SEQ_EVENT_SETPOS_TICK] = "Set postition tick",
   [SND_SEQ_EVENT_SETPOS_TIME] = "Set position time",
   [SND_SEQ_EVENT_TEMPO] = "Set tempo",
   [SND_SEQ_EVENT_CLOCK] = "Clock",
   [SND_SEQ_EVENT_TICK] = "Tick",
   [SND_SEQ_EVENT_QUEUE_SKEW] = "Queue skew",
   [SND_SEQ_EVENT_SYNC_POS] = "Sync Position",
   [SND_SEQ_EVENT_TUNE_REQUEST] = "Tune Request",
   [SND_SEQ_EVENT_RESET] = "Reset",
   [SND_SEQ_EVENT_SENSING] = "Sensing",
   [43] = "43",
   [44] = "44",
   [45] = "45",
   [46] = "46",
   [47] = "47",
   [48] = "48",
   [49] = "49",
   [SND_SEQ_EVENT_ECHO] = "Echo",
   [SND_SEQ_EVENT_OSS] = "OSS emulation raw event",
   [52] = "52",
   [53] = "53",
   [54] = "54",
   [55] = "55",
   [56] = "56",
   [57] = "57",
   [58] = "58",
   [59] = "59",
   [SND_SEQ_EVENT_CLIENT_START] = "Client Start",
   [SND_SEQ_EVENT_CLIENT_EXIT] = "Client Exit",
   [SND_SEQ_EVENT_CLIENT_CHANGE] = "Client Change",
   [SND_SEQ_EVENT_PORT_START] = "Port Start",
   [SND_SEQ_EVENT_PORT_EXIT] = "Port Exit",
   [SND_SEQ_EVENT_PORT_CHANGE] = "Port Change",
   [SND_SEQ_EVENT_PORT_SUBSCRIBED] = "Port Subscribed",
   [SND_SEQ_EVENT_PORT_UNSUBSCRIBED] = "Port Unsubscribed",
   [68] = "68",
   [69] = "69",
   [70] = "70",
   [71] = "71",
   [72] = "72",
   [73] = "73",
   [74] = "74",
   [75] = "75",
   [76] = "76",
   [77] = "77",
   [78] = "78",
   [79] = "79",
   [80] = "80",
   [81] = "81",
   [82] = "82",
   [83] = "83",
   [84] = "84",
   [85] = "85",
   [86] = "86",
   [87] = "87",
   [88] = "88",
   [89] = "89",
   [SND_SEQ_EVENT_USR0] = "User 0",
   [SND_SEQ_EVENT_USR1] = "User 1",
   [SND_SEQ_EVENT_USR2] = "User 2",
   [SND_SEQ_EVENT_USR3] = "User 3",
   [SND_SEQ_EVENT_USR4] = "User 4",
   [SND_SEQ_EVENT_USR5] = "User 5",
   [SND_SEQ_EVENT_USR6] = "User 6",
   [SND_SEQ_EVENT_USR7] = "User 7",
   [SND_SEQ_EVENT_USR8] = "User 8",
   [SND_SEQ_EVENT_USR9] = "User 9",
   [100] = "100",
   [101] = "101",
   [102] = "102",
   [103] = "103",
   [104] = "104",
   [105] = "105",
   [106] = "106",
   [107] = "107",
   [108] = "108",
   [109] = "109",
   [110] = "110",
   [111] = "111",
   [112] = "112",
   [113] = "113",
   [114] = "114",
   [115] = "115",
   [116] = "116",
   [117] = "117",
   [118] = "118",
   [119] = "119",
   [120] = "120",
   [121] = "121",
   [122] = "122",
   [123] = "123",
   [124] = "124",
   [125] = "125",
   [126] = "126",
   [127] = "127",
   [128] = "128",
   [129] = "129",
   [SND_SEQ_EVENT_SYSEX] = "System Exclusive",
   [SND_SEQ_EVENT_BOUNCE] = "Bounce",
   [132] = "132",
   [133] = "133",
   [134] = "134",
   [SND_SEQ_EVENT_USR_VAR0] = "Var 0",
   [SND_SEQ_EVENT_USR_VAR1] = "Var 1",
   [SND_SEQ_EVENT_USR_VAR2] = "Var 2",
   [SND_SEQ_EVENT_USR_VAR3] = "Var 3",
   [SND_SEQ_EVENT_USR_VAR4] = "Var 4",
   [140] = "140",
   [141] = "141",
   [142] = "142",
   [143] = "143",
   [144] = "144",
   [145] = "145",
   [146] = "146",
   [147] = "147",
   [148] = "148",
   [149] = "149",
   [150] = "150",
   [151] = "151",
   [152] = "152",
   [153] = "153",
   [154] = "154",
   [155] = "155",
   [156] = "156",
   [157] = "157",
   [158] = "158",
   [159] = "159",
   [160] = "160",
   [161] = "161",
   [162] = "162",
   [163] = "163",
   [164] = "164",
   [165] = "165",
   [166] = "166",
   [167] = "167",
   [168] = "168",
   [169] = "169",
   [170] = "170",
   [171] = "171",
   [172] = "172",
   [173] = "173",
   [174] = "174",
   [175] = "175",
   [176] = "176",
   [177] = "177",
   [178] = "178",
   [179] = "179",
   [180] = "180",
   [181] = "181",
   [182] = "182",
   [183] = "183",
   [184] = "184",
   [185] = "185",
   [186] = "186",
   [187] = "187",
   [188] = "188",
   [189] = "189",
   [190] = "190",
   [191] = "191",
   [192] = "192",
   [193] = "193",
   [194] = "194",
   [195] = "195",
   [196] = "196",
   [197] = "197",
   [198] = "198",
   [199] = "199",
   [200] = "200",
   [201] = "201",
   [202] = "202",
   [203] = "203",
   [204] = "204",
   [205] = "205",
   [206] = "206",
   [207] = "207",
   [208] = "208",
   [209] = "209",
   [210] = "210",
   [211] = "211",
   [212] = "212",
   [213] = "213",
   [214] = "214",
   [215] = "215",
   [216] = "216",
   [217] = "217",
   [218] = "218",
   [219] = "219",
   [220] = "220",
   [221] = "221",
   [222] = "222",
   [223] = "223",
   [224] = "224",
   [225] = "225",
   [226] = "226",
   [227] = "227",
   [228] = "228",
   [229] = "229",
   [230] = "230",
   [231] = "231",
   [232] = "232",
   [233] = "233",
   [234] = "234",
   [235] = "235",
   [236] = "236",
   [237] = "237",
   [238] = "238",
   [239] = "239",
   [240] = "240",
   [241] = "241",
   [242] = "242",
   [243] = "243",
   [244] = "244",
   [245] = "245",
   [246] = "246",
   [247] = "247",
   [248] = "248",
   [249] = "249",
   [250] = "250",
   [251] = "251",
   [252] = "252",
   [253] = "253",
   [254] = "254",
   [SND_SEQ_EVENT_NONE] = "No operation",
  };

#ifndef DOC_HIDDEN
#define FIXED_EV(x)	(_SND_SEQ_TYPE(SND_SEQ_EVFLG_FIXED) | _SND_SEQ_TYPE(x))
#endif

inline void printalsatime(const snd_seq_timestamp_t * time,
                     bool asticks=false) {
  if (asticks)
    std::cerr << " tick=" << time->tick;
  else
    std::cerr << " " << time->time.tv_sec
              << " s + " << time->time.tv_nsec
              << " ns";
}

inline void printalsadata(unsigned char type,
                     const void * data,
                     size_t size) {
  switch (snd_seq_event_types[type]) {
  case  FIXED_EV(SND_SEQ_EVFLG_RESULT): {
    const snd_seq_result_t * result = static_cast<const snd_seq_result_t *>(data);
    std::cerr << "Processed " << result->event
              << " (" << (0 <= result->event && result->event < 255? alsatypenames[result->event]:"")
              << ") with result " << result->result;
    break;
  }
  case  FIXED_EV(SND_SEQ_EVFLG_NOTE) | _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_NOTE_TWOARG):
  case  FIXED_EV(SND_SEQ_EVFLG_NOTE): {
    const snd_seq_ev_note * note = static_cast<const snd_seq_ev_note *>(data);
    std::cerr << "Note ch=" << (int)note->channel
              << " note=" << (int)note->note
              << " vel=" << (int)note->velocity;
    if (snd_seq_event_types[type] & _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_NOTE_TWOARG))
      std::cerr << " off vel=" << (int)note->off_velocity
                << " dur=" << note->duration;
    break;
  }
  case  FIXED_EV(SND_SEQ_EVFLG_CONTROL): {
    const snd_seq_ev_ctrl * ctrl = static_cast<const snd_seq_ev_ctrl *>(data);
    std::cerr << "Ctrl ch=" << (unsigned int) ctrl->channel
              << "uu=" << std::hex << (unsigned int) ctrl->unused[0]
              << " "  << std::hex << (unsigned int) ctrl->unused[1]
              << " "  << std::hex << (unsigned int) ctrl->unused[2] << std::dec
              << " param=" << ctrl->param
              << " value=" << ctrl->value;
    break;
  }
  case  FIXED_EV(SND_SEQ_EVFLG_QUEUE):
  case  FIXED_EV(SND_SEQ_EVFLG_QUEUE) | _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TICK):
  case  FIXED_EV(SND_SEQ_EVFLG_QUEUE) | _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TIME):
  case  FIXED_EV(SND_SEQ_EVFLG_QUEUE) | _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_VALUE): {
    const snd_seq_ev_queue_control * ctrl = static_cast<const snd_seq_ev_queue_control *>(data);
    std::cerr << "Queue " << (unsigned int) ctrl->queue;
    switch (snd_seq_event_types[type] &
            ( _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TICK) |
              _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TIME) |
              _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_VALUE))) {
    case _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TIME):
      std::cerr << " time=";
      printalsatime(&(ctrl->param.time), false);
      break;
    case _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_TICK):
      std::cerr << " time=";
      printalsatime(&(ctrl->param.time), true);
      break;
    case _SND_SEQ_TYPE_OPT(SND_SEQ_EVFLG_QUEUE_VALUE):
    default:
      std::cerr << " value=" << ctrl->param.value;
      std::cerr << " pos=" << ctrl->param.position
                << " skew=" << ctrl->param.skew.value
                << " sbase=" << ctrl->param.skew.base
                << " d32=" << ctrl->param.d32
                << " d8=" << ctrl->param.d8;
    }
    break;
  }
  case  FIXED_EV(SND_SEQ_EVFLG_MESSAGE): {
    const snd_seq_addr_t * addr = static_cast<const snd_seq_addr_t *>(data);
    std::cerr << "Address " << addr->client << ":" << addr->port;
    break;
  }
  case  FIXED_EV(SND_SEQ_EVFLG_CONNECTION): {
    const snd_seq_connect_t * conection = static_cast<const snd_seq_connect_t *>(data);
    std::cerr << "Connection "
              << conection->sender.client << ":"
              << conection->sender.port << " ==> "
              << conection->dest.client << ":"
              << conection->dest.port;
    break;
  }
  case  _SND_SEQ_TYPE(SND_SEQ_EVFLG_VARIABLE):
  case  _SND_SEQ_TYPE(SND_SEQ_EVFLG_VARIABLE) | _SND_SEQ_TYPE(SND_SEQ_EVFLG_USERS): {
    const snd_seq_ev_ext_t * evt = static_cast<const snd_seq_ev_ext_t *>(data);
    std::cerr << "length " << evt->len << ":";
    for (size_t i = 0 ; i < evt->len; ++i)
      std::cerr << " " << std::hex << (unsigned int) ((unsigned char *)evt->ptr)[i] << std::dec;
    break;
  }

  case  FIXED_EV(SND_SEQ_EVFLG_RAW) | FIXED_EV(SND_SEQ_EVFLG_USERS):
  case  FIXED_EV(SND_SEQ_EVFLG_RAW) | FIXED_EV(SND_SEQ_EVFLG_SYSTEM):
  case  FIXED_EV(SND_SEQ_EVFLG_NONE):
  default:
  for (size_t i = 0 ; i < size ; ++i)
    std::cerr << std::hex << (unsigned int) *((unsigned char *)(&(data))) << " ";
  }
  std::cerr << std::dec;
}


void printalsaevent(const snd_seq_event_t * ev) {
  std::cerr << "ALSA event {"
            << "\n  Type = " << std::dec << (unsigned int)ev->type
            << ":" << alsatypenames[ev->type]   // snd_seq_event_type_t
            << "\n  Flags = " << std::oct << (unsigned int)ev->flags  << std::dec
            << "\n  tag = " << (unsigned int) ev->tag
            << "\n  queue = " << (unsigned int)ev->queue
            << "\n  time = ";
  printalsatime(&(ev->time));                // snd_seq_timestapm_t
  std::cerr << "\n  path = " << (int)ev->source.client << ":" << (int)ev->source.port
            << " ==> " << (int)ev->dest.client << ":" << (int)ev->dest.port
            << "\n data = ";
  printalsadata(ev->type,&(ev->data),sizeof(ev->data));
  std::cerr << "\n}" << std::endl;
}


template<int locking>
void AlsaSequencer<locking>::inputThread() throw() {
  static const std::chrono::microseconds sleeptime( 10 );
  snd_seq_event_t * ev;
  int poll_fd_count;
  struct pollfd * poll_fds;
  int result;


  threadStatus = THREAD_START;

  poll_fd_count = snd_seq_poll_descriptors_count( seq, POLLIN ) + 1;
  poll_fds = (struct pollfd*) alloca( poll_fd_count * sizeof( struct pollfd ) );
  snd_seq_poll_descriptors( seq, poll_fds + 1, poll_fd_count - 1, POLLIN );
  poll_fds[0].fd = trigger_fds[0];
  poll_fds[0].events = POLLIN;

  {
    std::lock_guard< std::mutex > lock(syncMutex);
    threadStatus = THREAD_STARTED;
    syncThread.notify_all();
  }

  while ( threadCommand == THREAD_RUN ) {
    {
      int errcode;
      {
        std::lock_guard<std::mutex> lock2(mutex);
        errcode = snd_seq_event_input_pending( seq, 1 );
      }
      if ( errcode == 0 ) {
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
    }

    // If here, there should be data.
    for (;;) {
      {
#if 0
        scoped_lock<locking> lock ( mutex );
#else
        std::lock_guard<std::mutex> lock ( mutex );
#endif
        result = snd_seq_event_input( seq, &ev );
      }
      if (result != -EAGAIN && result != -EWOULDBLOCK)
        break;
      std::this_thread::sleep_for(sleeptime);
    }
#if 0
    printalsaevent(ev);
#endif
    if ( result == -ENOSPC ) {
      try {
        error( RTMIDI_ERROR( rtmidi_gettext( "ALSA MIDI input buffer overrun." ),
                                   Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }

      continue;
    }
    else if ( result == -EAGAIN ) {
      try {
        error( RTMIDI_ERROR( rtmidi_gettext( "ALSA returned without providing a MIDI event." ),
                                   Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }

      continue;
    }
    else if ( result <= 0 ) {
      try {
        error( RTMIDI_ERROR1( rtmidi_gettext( "Unknown ALSA MIDI input error.\nThe system reports:\n%s" ),
                                    Error::WARNING,
                                    strerror( -result ) ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }
      continue;
    }

    {
      std::unique_lock<portlist_t> lock(ports);
      if (ev->dest.port >= ports.size()) {
        error( RTMIDI_ERROR1( rtmidi_gettext( "Received an ALSA event with an invalid port number (%d)." ),
                              Error::WARNING,
                              ev->dest.port ),
               lock);
        continue;
      }
      MidiAlsa * port = dynamic_cast<MidiAlsa *>(ports[ev->dest.port]);
      if (port) {
        port -> receiveEvent(ev);
      } else {
        error( RTMIDI_ERROR1( rtmidi_gettext( "Received an ALSA event with an invalid port number (%d)." ),
                              Error::WARNING,
                              ev->dest.port ),
               lock);
      }
    }
    snd_seq_free_event( ev );
  }

  {
    std::unique_lock<std::mutex> lock(syncMutex);
    threadStatus = THREAD_STOP;
    syncThread.notify_all();
  }
}
#undef RTMIDI_CLASSNAME

#define RTMIDI_CLASSNAME "AlsaPortDescriptor"

std::shared_ptr<LockingAlsaSequencer> AlsaPortDescriptor :: main_seq;

PortList AlsaPortDescriptor :: getPortList( int capabilities, std::shared_ptr<LockingAlsaSequencer> & seq )
{
  PortList list;
  snd_seq_client_info_t * cinfo;
  snd_seq_port_info_t * pinfo;
  int client;

  if ( !( capabilities & INOUTPUT ) )
    return list;

  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  snd_seq_client_info_set_client( cinfo, -1 );
  while ( seq->getNextClient( cinfo ) >= 0 ) {
    client = snd_seq_client_info_get_client( cinfo );
    // ignore default device ( it is included in the following results again )
    if ( client == 0 ) continue;
    // Reset query info
    snd_seq_port_info_set_client( pinfo, client );
    snd_seq_port_info_set_port( pinfo, -1 );
    while ( seq->getNextPort( pinfo ) >= 0 ) {
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
                                              new AlsaPortDescriptor( client, snd_seq_port_info_get_port( pinfo ), seq ) ) );
    }
  }
  return list;
}

InternalMidiApi * AlsaPortDescriptor :: getApi ( unsigned int capabilities ) const {
  // either input or output or both can be handled by the same API class
  if ( capabilities & INOUTPUT ) {
    MidiAlsa * api = new MidiAlsa( "" );
#if 0
    if (api) api->setCurrentCapabilities( capabilities );
#endif
    return api;
  } else
    return nullptr;
}

#undef RTMIDI_CLASSNAME


#define RTMIDI_CLASSNAME "MidiAlsa"
void MidiAlsa :: receiveEvent( snd_seq_event_t * ev ) throw( )
{
  // This is a bit weird, but we now have to decode an ALSA MIDI
  // event ( back ) into MIDI bytes. We'll ignore non-MIDI types.
  // TODO: provide an event based API

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
    return;
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
    return;
    break;

  case SND_SEQ_EVENT_QFRAME: // MIDI time code
  case SND_SEQ_EVENT_TICK: // 0xF9 ... MIDI timing tick
  case SND_SEQ_EVENT_CLOCK: // 0xF8 ... MIDI timing ( clock ) tick
    if ( ( ignoreFlags & IGNORE_TIME ) )
      return;
    break;

  case SND_SEQ_EVENT_SENSING: // Active sensing
    if ( ( ignoreFlags & IGNORE_SENSING ) )
      return;
    break;

  case SND_SEQ_EVENT_SYSEX:
    if ( ( ignoreFlags & IGNORE_SYSEX ) )
      return;
    // decode message directly into the buffer

    // The ALSA sequencer has a maximum buffer size for MIDI sysex
    // events of 256 bytes. If a device sends sysex messages larger
    // than this, they are segmented into 256 byte chunks. So,
    // we'll watch for this and concatenate sysex chunks into a
    // single sysex message if necessary.
    try {
      doSysExCallback( makeTimeStamp ( ev ),
                       static_cast<unsigned char * >(ev->data.ext.ptr),
                       ev->data.ext.len,
                       ev->source.client << 8 | ev->source.port );
#if 0
      rcv_sysex_size =


        doSysEx( ev,
                          rcv_sysex_size,
                          currentMessage );
#endif
    } catch ( std::bad_alloc& e ) {
      try {
        error( RTMIDI_ERROR( rtmidi_gettext( "Error resizing buffer memory." ),
                             Error::WARNING ) );
      } catch ( Error& e ) {
        // don't bother ALSA with an unhandled exception
      }
    }
    return;
    break;
  }

  doAlsaEvent( ev );
}

MidiAlsa :: MidiAlsa( const std::string& clientName )
  : AlsaMidi ( clientName ),
    InternalMidiApi( )
{
  MidiAlsa::initialize( clientName );
}

MidiAlsa :: ~MidiAlsa( )
{
  // Close a connection if it exists.
  MidiAlsa::closePort( );


  // Cleanup.
  // TODO: Merge with AlsaMidi
  try {
    if ( local.client && local.port )
      deletePort( );
  } catch ( const Error& e ) {
    // we don't have access to the error handler
    e.printMessage( );
  }
}



#if 0
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
#endif


Pointer<PortDescriptor> MidiAlsa :: getDescriptor( bool isLocal )
{
  try {
    if ( isLocal ) {
      if ( local.client ) {
        return Pointer<PortDescriptor>( new AlsaPortDescriptor( local, seq) );
      }
    } else {
      if ( client ) {
        return Pointer<PortDescriptor>( new AlsaPortDescriptor( *this, seq ) );
      }
    }
  } catch ( Error & e ) {
    error ( e );
  }
  return NULL;
}

PortList MidiAlsa :: getPortList( int capabilities )
{
  try {
    return AlsaPortDescriptor::getPortList( capabilities,
                                            seq );
  } catch ( Error& e ) {
    error ( e );
    return PortList( );
  }
}



void MidiAlsa :: openVirtualPort( const std::string& portName,
                                  unsigned int capabilities )
{
  if ( !local.client ) {
    if ( createPort( capabilities2ALSA( capabilities ), portName ) ) {
      //errorString_ = "MidiInAlsa::openVirtualPort: ";
      error( RTMIDI_ERROR( gettext_noopt( "Error creating ALSA port." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
  }
  if (capabilities & OUTPUT && !out_coder) {
    try {
      createCoder ( &out_coder );
    } catch ( const Error & e ) {
      closePort ();
      error ( e );
    }
  }
  if (capabilities & INPUT && !in_coder) {
    try {
      createCoder ( &in_coder );
#ifdef RTMIDI_DEBUG
      snd_midi_event_no_status( in_coder, 0 ); // debug running status messages
#else
      snd_midi_event_no_status( in_coder, 1 ); // suppress running status messages
#endif
    } catch ( const Error & e ) {
      closePort ();
      error ( e );
    }
  }

#if 0
  if ( doInput == false ) {
    // Wait for old thread to stop, if still running
    if ( !pthread_equal( thread, dummy_thread_id ) )
      pthread_join( thread, NULL );

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
#endif
}

void MidiAlsa :: openPort( const PortDescriptor& port,
                             const std::string& portName,
                             unsigned int capabilities )
{
  const AlsaPortDescriptor * remote = dynamic_cast<const AlsaPortDescriptor *>( &port );

  if ( connected_ ) {
    error( RTMIDI_ERROR( gettext_noopt( "A valid connection already exists." ),
                         Error::WARNING ) );
    return;
  }
  if ( !remote ) {
    error( RTMIDI_ERROR( gettext_noopt( "ALSA has been instructed to open a non-ALSA MIDI port. This doesn't work." ),
                         Error::INVALID_DEVICE ) );
    return;
  }

  if ( !local.client )
    openVirtualPort ( portName, capabilities );

  try {
    setRemote( remote );
    connectPorts( *remote,
                  local,
                  capabilities );


    connected_ = true;
  } catch ( Error& e ) {
    error( e );
  }
}

size_t MidiAlsa :: maxSysExSize() const {
  return base::maxSysExSize();
}

void MidiAlsa :: closePort( void )
{
    AlsaMidi :: closePort();
#if 0
    if ( subscription_in ) {
      closePort( subscription_in);
      subscription_in = 0;
      snd_seq_unsubscribe_port( seq, subscription );
      snd_seq_port_subscribe_free( subscription );
      subscription = 0;
    }
    // Stop the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_stop_queue( seq, queue_id, NULL );
    snd_seq_drain_output( seq );
#endif
#endif

#if 0
  // Stop thread to avoid triggering the callback, while the port is intended to be closed
  if ( doInput ) {
    doInput = false;
    int res = write( trigger_fds[1], &doInput, sizeof( doInput ) );
    ( void ) res;
    if ( !pthread_equal( thread, dummy_thread_id ) )
      pthread_join( thread, NULL );
  }
#endif
  connected_ = false;
}


void MidiAlsa :: sendMessage( const unsigned char * message, size_t size )
{
  long result;
  #if 0
  if (size > 4) {
    result = snd_midi_event_resize_buffer ( out_coder, size );
    if ( result != 0 ) {
      error( RTMIDI_ERROR( gettext_noopt( "ALSA error resizing MIDI event buffer." ),
                           Error::DRIVER_ERROR ) );
      return;
    }
  }
  #endif

  snd_seq_event_t ev;
  snd_seq_ev_clear( &ev );
  snd_seq_ev_set_source( &ev, local.port );
  snd_seq_ev_set_subs( &ev );
  snd_seq_ev_set_direct( &ev );

  auto startsize = size;
  int count = 0;

  // In case there are more messages in the stream we send everything
  while ( size && ( result = snd_midi_event_encode( out_coder,
                                                    message,
                                                    size, &ev ) ) > 0 ) {
    ++count;
    seq->sendMessage(ev);
    if ( size < (size_t) result ) {
      error( RTMIDI_ERROR( gettext_noopt( "ALSA consumed more bytes than availlable." ),
                           Error::WARNING ) );
      return;
    }
    message += result;
    size -= result;
  }
}

void MidiAlsa :: initialize( const std::string& clientName )
{
  init( );
  if (!clientName.empty()) {
    setClientName( clientName );
  }
}
#undef RTMIDI_CLASSNAME

//*********************************************************************//
// API: LINUX ALSA
// Class Definitions: MidiOutAlsa
//*********************************************************************//

#define RTMIDI_CLASSNAME "MidiOutAlsa"
#if 0
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

#endif



#undef RTMIDI_CLASSNAME
RTMIDI_NAMESPACE_END

#endif // __LINUX_ALSA__

