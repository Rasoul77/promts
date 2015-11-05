#ifndef TIME_FUNCTIONS_H
#define TIME_FUNCTIONS_H

#include <ctime>
#include <sys/time.h>

namespace rasoul{
  namespace common{

  // usage:
  //     common::timestamp_t t0, t1;
  //     t0 = common::get_timestamp();
  //     perform_search();
  //     t1 = common::get_timestamp();
  //     double dt = (double) (t1 - t0) / 1000000.0L;
  //     cout << "Search Time = " << dt << " Seconds";

  typedef unsigned long long timestamp_t;
  void wait(double seconds);
  timestamp_t get_timestamp (void);

  }; // namespace common
}; // namespace rasoul
#endif
