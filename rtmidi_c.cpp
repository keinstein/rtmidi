#include <string.h>
#include <stdlib.h>
#include "rtmidi_c.h"
#define RTMIDI_NO_WARN_DEPRECATED 1
#include "RtMidi.h"

#define RTMIDI_CLASSNAME "C interface"

/* Compile-time assertions that will break if the enums are changed in
 * the future without synchronizing them properly.  If you get (g++)
 * "error: ‘StaticAssert<b>::StaticAssert() [with bool b = false]’ is
 * private within this context", it means enums are not aligned. */
template<bool b> class StaticAssert { private: StaticAssert() {} };
template<> class StaticAssert<true>{ public: StaticAssert() {} };
#define ENUM_EQUAL(x,y) StaticAssert<(int)x==(int)y>()
class StaticAssertions { StaticAssertions() {
  StaticAssert<sizeof(enum RtMidiApi) == sizeof(rtmidi::ApiType)>();
  ENUM_EQUAL( RT_MIDI_API_UNSPECIFIED,     rtmidi::UNSPECIFIED );
  ENUM_EQUAL( RT_MIDI_API_MACOSX_CORE,     rtmidi::MACOSX_CORE );
  ENUM_EQUAL( RT_MIDI_API_LINUX_ALSA,      rtmidi::LINUX_ALSA );
  ENUM_EQUAL( RT_MIDI_API_UNIX_JACK,       rtmidi::UNIX_JACK );
  ENUM_EQUAL( RT_MIDI_API_WINDOWS_MM,      rtmidi::WINDOWS_MM );
  ENUM_EQUAL( RT_MIDI_API_RTMIDI_DUMMY,    rtmidi::DUMMY );
  ENUM_EQUAL( RT_MIDI_API_WINDOWS_KS,      rtmidi::WINDOWS_KS );
  ENUM_EQUAL( RT_MIDI_API_ALL_API,         rtmidi::ALL_API );
  ENUM_EQUAL( RT_MIDI_API_NUM,             rtmidi::NUM_APIS);

  ENUM_EQUAL( RT_ERROR_WARNING,            rtmidi::Error::WARNING );
  ENUM_EQUAL( RT_ERROR_DEBUG_WARNING,      rtmidi::Error::DEBUG_WARNING );
  ENUM_EQUAL( RT_ERROR_UNSPECIFIED,        rtmidi::Error::UNSPECIFIED );
  ENUM_EQUAL( RT_ERROR_NO_DEVICES_FOUND,   rtmidi::Error::NO_DEVICES_FOUND );
  ENUM_EQUAL( RT_ERROR_INVALID_DEVICE,     rtmidi::Error::INVALID_DEVICE );
  ENUM_EQUAL( RT_ERROR_MEMORY_ERROR,       rtmidi::Error::MEMORY_ERROR );
  ENUM_EQUAL( RT_ERROR_INVALID_PARAMETER,  rtmidi::Error::INVALID_PARAMETER );
  ENUM_EQUAL( RT_ERROR_INVALID_USE,        rtmidi::Error::INVALID_USE );
  ENUM_EQUAL( RT_ERROR_DRIVER_ERROR,       rtmidi::Error::DRIVER_ERROR );
  ENUM_EQUAL( RT_ERROR_SYSTEM_ERROR,       rtmidi::Error::SYSTEM_ERROR );
  ENUM_EQUAL( RT_ERROR_THREAD_ERROR,       rtmidi::Error::THREAD_ERROR );
}};

class CallbackProxyUserData
{
  public:
	CallbackProxyUserData (RtMidiCCallback cCallback, void *userData)
		: c_callback (cCallback), user_data (userData)
	{
	}
	RtMidiCCallback c_callback;
	void *user_data;
};

extern "C" const enum rtmidi::ApiType rtmidi_compiled_system_apis[]; // casting from RtMidi::Api[]
extern "C" const unsigned int rtmidi_num_compiled_system_apis;
extern "C" enum rtmidi::ApiType rtmidi_compiled_software_apis[]; // casting from RtMidi::Api[]
extern "C" const unsigned int rtmidi_num_compiled_software_apis;
extern "C" const enum rtmidi::ApiType rtmidi_compiled_other_apis[]; // casting from RtMidi::Api[]
extern "C" const unsigned int rtmidi_num_compiled_other_apis;

/* RtMidi API */
int rtmidi_get_compiled_api (enum RtMidiApi *apis, unsigned int apis_size)
{
  unsigned int retval = rtmidi_num_compiled_system_apis +
    rtmidi_num_compiled_software_apis +
    rtmidi_num_compiled_other_apis;
  unsigned num = rtmidi_num_compiled_system_apis;
  if (apis) {
    retval = retval > apis_size ? apis_size: retval;
    num = (num < apis_size) ? num : apis_size;
    memcpy(apis, rtmidi_compiled_system_apis, num * sizeof(enum RtMidiApi));
    apis_size -= num;
    apis += num;
    if (apis_size) {
      num = rtmidi_num_compiled_software_apis;
      num = (num < apis_size) ? num : apis_size;
      memcpy(apis, rtmidi_compiled_software_apis, num * sizeof(enum RtMidiApi));
      apis_size -= num;
      apis += num;
    }
    if (apis_size) {
      num = rtmidi_num_compiled_other_apis;
      num = (num < apis_size) ? num : apis_size;
      memcpy(apis, rtmidi_compiled_other_apis, num * sizeof(enum RtMidiApi));
      apis_size -= num;
    }
  }
  return (int)retval;
}

extern "C"
struct api_name {
  rtmidi::ApiType Id;
  const char * machine;
  const char * display;
};

extern "C" const api_name rtmidi_api_names[];
const char *rtmidi_api_name(enum RtMidiApi api) {
    if (api < 0 || api >= RT_MIDI_API_NUM)
        return NULL;
    return rtmidi_api_names[api].machine;
}

const char *rtmidi_api_display_name(enum RtMidiApi api)
{
    if (api < 0 || api >= RT_MIDI_API_NUM)
        return "Unknown";
    return rtmidi_api_names[api].display;
}

enum RtMidiApi rtmidi_compiled_api_by_name(const char *name) {
    rtmidi::ApiType api = rtmidi::UNSPECIFIED;
    if (name) {
      api = rtmidi::Midi::getCompiledApiByName(name);
    }
    return (enum RtMidiApi)api;
}

void rtmidi_error (rtmidi::MidiApi *api, enum RtMidiErrorType type, const char* errorString)
{
	// std::string msg = errorString;
	api->error (RTMIDI_ERROR(errorString,(rtmidi::Error::Type)type)); /* (RtMidiError::Type) type, msg); */
}

void rtmidi_open_port (RtMidiPtr device, unsigned int portNumber, const char *portName)
{
    std::string name = portName;
    try {
      (device->ptr)->openPort (portNumber, name, device->flags);

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
    }
}

void rtmidi_open_virtual_port (RtMidiPtr device, const char *portName)
{
    std::string name = portName;
    try {
	((rtmidi::RtMidiCompatibility*)device->ptr)->openVirtualPort (name);

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
    }

}

void rtmidi_close_port (RtMidiPtr device)
{
    try {
	((rtmidi::Midi*)device->ptr)->closePort ();

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
    }
}

unsigned int rtmidi_get_port_count (RtMidiPtr device)
{
    try {
	return ((rtmidi::RtMidiCompatibility *)device->ptr)->getPortCount ();

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
        return -1;
    }
}

const char* rtmidi_get_port_name (RtMidiPtr device, unsigned int portNumber)
{
    try {
	std::string name = ((rtmidi::RtMidiCompatibility*)device->ptr)->getPortName (portNumber);
        return strdup (name.c_str ());

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
        return "";
    }
}

/* MidiIn API */
RtMidiPtr rtmidi_in_create_default ()
{
    RtMidiPtr wrp = new RtMidiWrapper;

    if (!wrp) return wrp;
    try {
        wrp->ptr = new RtMidiIn ();
        wrp->flags = rtmidi::PortDescriptor::INPUT;
        wrp->data = 0;
        wrp->ok  = true;
        wrp->msg = "";

    } catch (const RtMidiError & err) {
        wrp->ptr = 0;
        wrp->data = 0;
        wrp->ok  = false;
        wrp->msg = err.what ();
    }

    return wrp;
}

RtMidiPtr rtmidi_in_create (enum RtMidiApi api, const char *clientName, unsigned int queueSizeLimit)
{
    std::string name = clientName;
    RtMidiWrapper* wrp = new RtMidiWrapper;

    try {
        RtMidiIn* rIn = new RtMidiIn ((rtmidi::ApiType) api, name, queueSizeLimit);

        wrp->ptr = rIn;
        wrp->flags = rtmidi::PortDescriptor::INPUT;
        wrp->data = 0;
        wrp->ok  = true;
        wrp->msg = "";

    } catch (const RtMidiError & err) {
        wrp->ptr = 0;
        wrp->data = 0;
        wrp->ok  = false;
        wrp->msg = err.what ();
    }

    return wrp;
}

void rtmidi_in_free (RtMidiPtr device)
{
    if (device->data)
      delete (CallbackProxyUserData*) device->data;
    delete (rtmidi::MidiIn*) device->ptr;
    delete device;
}

enum RtMidiApi rtmidi_in_get_current_api (RtMidiPtr device)
{
    try {
	return (RtMidiApi) (device->ptr)->getCurrentApi ();

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();

        return RT_MIDI_API_UNSPECIFIED;
    }
}

static
void callback_proxy (double timeStamp, std::vector<unsigned char> *message, void *userData)
{
	CallbackProxyUserData* data = reinterpret_cast<CallbackProxyUserData*> (userData);
	data->c_callback (timeStamp, message->data (), message->size (), data->user_data);
}

void rtmidi_in_set_callback (RtMidiPtr device, RtMidiCCallback callback, void *userData)
{
    device->data = (void*) new CallbackProxyUserData (callback, userData);
    try {
	((rtmidi::MidiIn*)device->ptr)->setCallback (callback_proxy, device->data);
    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
        delete (CallbackProxyUserData*) device->data;
        device->data = 0;
    }
}

void rtmidi_in_cancel_callback (RtMidiPtr device)
{
    try {
	((rtmidi::MidiIn*)device->ptr)->cancelCallback ();
        delete (CallbackProxyUserData*) device->data;
        device->data = 0;
    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
    }
}

void rtmidi_in_ignore_types (RtMidiPtr device, bool midiSysex, bool midiTime, bool midiSense)
{
	((rtmidi::MidiIn*)device->ptr)->ignoreTypes (midiSysex, midiTime, midiSense);
}

double rtmidi_in_get_message (RtMidiPtr device,
                              unsigned char *message,
                              size_t *size)
{
    try {
        // FIXME: use allocator to achieve efficient buffering
        std::vector<unsigned char> v;
        if (device->flags == rtmidi::PortDescriptor::INPUT) {
          RtMidiIn * dev = static_cast<RtMidiIn*>(device->ptr);
          double ret = dev->getMessage (v);

          if (v.size () > 0 && v.size() <= *size) {
            memcpy (message, v.data (), (int) v.size ());
          }

          *size = v.size();
          return ret;
        } else {
          *size = 0;
          return 0.0;
        }
    }
    catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
        return -1;
    }
    catch (...) {
        device->ok  = false;
        device->msg = "Unknown error";
        return -1;
    }
}

/* RtMidiOut API */
RtMidiPtr rtmidi_out_create_default ()
{
    RtMidiWrapper* wrp = new RtMidiWrapper;

    try {
	rtmidi::MidiOut* rOut = new rtmidi::MidiOut ();

        wrp->ptr = rOut;
        wrp->flags = rtmidi::PortDescriptor::OUTPUT;
        wrp->data = 0;
        wrp->ok  = true;
        wrp->msg = "";

    } catch (const RtMidiError & err) {
        wrp->ptr = 0;
        wrp->flags = rtmidi::PortDescriptor::OUTPUT;
        wrp->data = 0;
        wrp->ok  = false;
        wrp->msg = err.what ();
    }

    return wrp;
}

RtMidiPtr rtmidi_out_create (enum RtMidiApi api, const char *clientName)
{
    RtMidiWrapper* wrp = new RtMidiWrapper;
    std::string name = clientName;

    try {
        rtmidi::MidiOut* rOut = new rtmidi::MidiOut ((rtmidi::ApiType) api, name);

        wrp->ptr = rOut;
        wrp->flags = rtmidi::PortDescriptor::OUTPUT;
        wrp->data = 0;
        wrp->ok  = true;
        wrp->msg = "";

    } catch (const RtMidiError & err) {
        wrp->ptr = 0;
        wrp->flags = rtmidi::PortDescriptor::OUTPUT;
        wrp->data = 0;
        wrp->ok  = false;
        wrp->msg = err.what ();
    }


    return wrp;
}

void rtmidi_out_free (RtMidiPtr device)
{
    delete ((rtmidi::MidiOut*)device->ptr);
    delete device;
}

enum RtMidiApi rtmidi_out_get_current_api (RtMidiPtr device)
{
    try {
        return (RtMidiApi) ((rtmidi::MidiOut*)device->ptr)->getCurrentApi ();

    } catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();

        return RT_MIDI_API_UNSPECIFIED;
    }
}

int rtmidi_out_send_message (RtMidiPtr device, const unsigned char *message, int length)
{
    try {
        ((RtMidiOut*) device->ptr)->sendMessage (message, length);
        return 0;
    }
    catch (const RtMidiError & err) {
        device->ok  = false;
        device->msg = err.what ();
        return -1;
    }
    catch (...) {
        device->ok  = false;
        device->msg = "Unknown error";
        return -1;
    }
}
