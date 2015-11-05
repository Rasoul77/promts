#ifndef MISC_HPP
#define MISC_HPP

#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <vector>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <limits>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

namespace rasoul{
  namespace common{

enum ENUM_COLOR_CODE
{
    FG_DEFAULT       = 39,
    FG_BLACK         = 30,
    FG_RED           = 31,
    FG_GREEN         = 32,
    FG_YELLOW        = 33,
    FG_BLUE          = 34,
    FG_MAGENTA       = 35,
    FG_CYAN          = 36,
    FG_LIGHT_GRAY    = 37,
    FG_DARK_GRAY     = 90,
    FG_LIGHT_RED     = 91,
    FG_LIGHT_GREEN   = 92,
    FG_LIGHT_YELLOW  = 93,
    FG_LIGHT_BLUE    = 94,
    FG_LIGHT_MAGENTA = 95,
    FG_LIGHT_CYAN    = 96,
    FG_WHITE         = 97,
    BG_RED           = 41,
    BG_GREEN         = 42,
    BG_BLUE          = 44,
    BG_DEFAULT       = 49
};

std::string Color(ENUM_COLOR_CODE col_code);
int getchr();
int getkey();
bool KbHit(int& ch);

template <typename T>
std::string NumberToString ( T Number )
{
   std::ostringstream ss;
   ss << std::setprecision(10);
   ss << Number;
   return ss.str();
}

template <typename T>
T StringToNumber ( const std::string &Text )
{
   std::istringstream ss(Text);
   T result;
   return ss >> result ? result : 0;
}

template <class T>
const T& max ( const T& a, const T& b ) {
  return (a<b)?b:a;
}

template <class T>
const T& min ( const T& a, const T& b ) {
  return (a<b)?a:b;
}

template <class T>
const T& max ( const std::vector<T>& a , size_t& i_max) {
  if(a.empty()) return(0);
  i_max = 0;
  T max_value = a[i_max];
  for(size_t i=1; i<a.size(); i++)
    if(a > max_value){max_value = a; i_max = i;}
  return(max_value);
}

class Trim{
  public:
    Trim(){}
    Trim(const std::string& str){ str_ = str; }
    // trim from start
    std::string ltrim() {
      std::string s(str_);
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
      return s;
    }

    // trim from end
    std::string rtrim() {
      std::string s(str_);
      s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
      return s;
    }

    // trim from both ends
    std::string trim() {
      std::string s(str_);
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
      s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
      return s;
    }
  protected:
    std::string str_;
};

class PathFileString
{
  public:
    PathFileString(){}
    PathFileString(const std::string& path_file_string){string_ = path_file_string;}

    void
    setFilePath(const std::string& path_file_string){string_ = path_file_string;}

    std::string getFileName()
    {
      size_t found = string_.find_last_of("/\\");
      return string_.substr(found+1);
    }

    std::string getFileNameNoExt()
    {
      std::string filename = getFileName();
      size_t found = filename.find_last_of(".");
      return filename.substr(0,found);
    }

    std::string getExtension()
    {
      std::string filename = getFileName();
      size_t found = filename.find_last_of(".");
      return filename.substr(found+1);
    }

    std::string getFolderWithLastBackSlash()
    {
      size_t found = string_.find_last_of("/\\");
      return string_.substr(0,found+1);
    }

    std::string getFolder()
    {
      size_t found = string_.find_last_of("/\\");
      return string_.substr(0,found);
    }

  protected:
    std::string string_;
};

template<class T>
void
combinations(T r, T n, std::vector<T>& Cnr)
{
  std::vector<bool> v(n);
  std::fill(v.begin() + n - r, v.end(), true);

  do {
      for (T i = 0; i < n; ++i) {
          if (v[i]) {
              //std::cout << i << " ";
              Cnr.push_back(i);
          }
      }
      //std::cout << "\n";
  } while (std::next_permutation(v.begin(), v.end()));
//    std::cout << "------------------------------------\n";
//
//    for(int i=0; i<combinations.size()/r; i++)
//    {
//      for(int j=0; j<r; j++)
//        std::cout << combinations[i*r+j] << " ";
//      std::cout << std::endl;
//    }
}

template<class T>
void
Cnr(const std::vector<T>& D, unsigned int r, std::vector<T>& C)
{
  unsigned int n = D.size();
  C.clear();

  std::vector<bool> v(n);
  std::fill(v.begin() + r, v.end(), true);
  do {
    for (unsigned int i = 0; i < n; ++i)
      if (!v[i])
        C.push_back(D[i]);
  } while (std::next_permutation(v.begin(), v.end()));
}

void PressEnterToContinue();

  }; // namespace common
}; // namespace rasoul
#endif
