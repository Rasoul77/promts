#include <rasoul_common_pkg/TimeFunctions.hpp>

namespace rasoul{
  namespace common{

void wait(double seconds)
{
  if(seconds < 0.0) return;
  double start_time = (double) clock()/CLOCKS_PER_SEC;
  double end_time = start_time;
  while(end_time - start_time < seconds) end_time = (double) clock()/CLOCKS_PER_SEC;
}

timestamp_t get_timestamp (void)
{
  struct timeval now;
  gettimeofday (&now, NULL);
  return  now.tv_usec + (timestamp_t) now.tv_sec * 1000000;
}

  };
};
