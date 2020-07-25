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
#include "src/RtMidi-internal.h"
#include <iostream>
#include <cstdlib>
#include <condition_variable>
#include <thread>
#include <sstream>


#define rtmidi_abort								\
	std::cerr << __FILE__ << ":" << __LINE__ << ": rtmidi_aborting" << std::endl; \
	abort

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


using namespace rtmidi;

bool ok = false;

typedef CopyOnWrite<std::vector<int *>> testlist_t;
testlist_t testlist;

std::vector<int> data = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                          11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };

class status {
public:
  status():stat(0),m(),cond(){}
  status & operator ++ () {
    std::lock_guard<std::mutex> lock(m);
    stat++;
    cond.notify_all();
    return *this;
  }

  bool operator != (int i) { return stat != i; }
  bool operator == (int i) { return stat == i; }
  bool operator <= (int i) { return stat <= i; }
  void lock() { m.lock(); }
  void unlock() { m.unlock(); }
  template<typename Predicate>
  void wait(Predicate p) { cond.wait(m,p); }

  operator std::mutex&() {
    return m;
  }
protected:
  int stat;
  std::mutex m;
  std::condition_variable_any cond;
} t1,t2,t1pause,t2pause;

void check_testlist(const std::string & s) {
  std::stringstream out;
  auto e = testlist.end();
  for(testlist_t::locked_iterator i(testlist); i!=e ; ++i) {
    if (*i)
      out << (*(*i)) << " " << std::flush;
    else
      out <<  "nullptr " << std::flush;
  }
  std::string got = out.str();
  if (got != s) {
    std::cerr << "Content mismatch:" << std::endl
              << "Expected:" << std::endl
              << s << std::endl
              << "Got:" << std::endl
              << got << std::endl;
    abort();
  }
}

void thread1 () {
  testlist.add(0,&data[0]);
  testlist.add(1,&data[1]);
  ++t1;
  std::cout << "p" << std::endl;
  std::cout << 1 << std::endl;
  t2.wait([]() { return t2 == 1; });
  std::cout << "a" << std::endl;
  testlist.add(4,&data[4]);
  std::cout << "+" << std::endl;
  ++t1;
  std::cout << "p" << std::endl;
  SLEEP(200);
  std::cout << "r" << std::endl;
  testlist.remove(&data[1]);
  SLEEP(100);
  testlist.add(5,&data[6]);
  std::cout << "+" << std::endl;
  ++t1;
  t2.wait([]() {return t2 == 2; });
  testlist.add(6,&data[5]);
}

void thread2 () {
  std::cout << 2 << std::endl;
  t1.wait([]() { return t1 == 1; });
  std::cout << "b" << std::endl;
  std::cout << "l" << std::endl;
  ++t2;
  std::cout << "q" << std::endl;
  std::cout << 2 << std::endl;
  t1.wait([]() { return t1 == 2; });
  std::cout << "b" << std::endl;
  testlist.add(2,&data[2]);
  {
    testlist_t::lock_guard lock(testlist);
    SLEEP(500);
  }
  testlist.add(3,&data[3]);
  std::cout << "u" << std::endl;
  t1.wait([](){ return t1 == 3; });
  testlist.remove(&data[6]);
  ++t2;
}


int main( int /* argc */, char * /*argv*/[] )
{
  std::thread thr1(thread1), thr2(thread2);

  thr1.join();
  thr2.join();
  check_testlist("0 nullptr 2 3 4 nullptr 5 ");
}
