#ifndef RANDFUNCTIONS_H
#define RANDFUNCTIONS_H

#include <stdlib.h>
#include <string.h>
#include <time.h>

namespace rasoul{
  namespace common{

  //flags for the extended randstr function
  #define RAND_STR_CAPITOL_LETTERS    0x00000001    //65 through 90
  #define RAND_STR_LOWER_CASE_LETTERS    0x00000002    //97 through 122
  #define    RAND_STR_NUMBERS        0x00000004    //48 through 57
  #define RAND_STR_SYMBOLS_1        0x00000008    //32 through 47
  #define RAND_STR_SYMBOLS_2        0x00000010    //58 through 64
  #define RAND_STR_SYMBOLS_3        0x00000020    //91 through 96
  #define RAND_STR_SYMBOLS_4        0x00000040    //123 through 126
  #define RAND_STR_NON_PRINTING        0x00000080    //1 through 31 and 127
  #define RAND_STR_EXTENDED_ASCII        0x00000100    //128 through 255

  //use this first function to seed the random number generator,
  //call this before any of the other functions
  void initrand();

  //generates a psuedo-random integer between 0 and 32767
  int randint();

  //generates a psuedo-random integer between 0 and max
  int randint(int max);

  //generates a psuedo-random integer between min and max
  int randint(int min, int max);

  //generates a psuedo-random float between 0.0 and 0.999...
  float randfloat();

  //generates a psuedo-random float between 0.0 and max
  float randfloat(float max);

  //generates a psuedo-random float between min and max
  float randfloat(float min, float max);

  //generates a psuedo-random double between 0.0 and 0.999...
  double randdouble();

  //generates a psuedo-random double between 0.0 and max
  double randdouble(double max);

  //generates a psuedo-random double between min and max
  double randdouble(double min, double max);

  char* randstr(char* str, unsigned long length);
  char* randstr(char* str, unsigned long length, unsigned long flags);

  }; // namespace common
}; // namespace rasoul
#endif
