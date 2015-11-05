#include "rasoul_common_pkg/Misc.hpp"

namespace rasoul{
  namespace common{

//////////////////////////////////////////////////////////////////////////////
// FUNCTION: PressEnterToContinue()
void PressEnterToContinue()
{
  std::cerr << "Press ENTER to continue... " << std::flush;
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FUNCTION: Color()
std::string Color(ENUM_COLOR_CODE col_code)
{
  std::stringstream ss;
  ss << "\033[" << col_code << "m";
  return(ss.str());
}
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FUNCTION: getchr()
int getchr()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FUNCTION: getkey()
int getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    character = fgetc(stdin);

    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FUNCTION: KbHit()
bool
KbHit(int& ch)
{
  struct termios oldt, newt;
  //int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchr();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    //ungetc(ch, stdin);
    return true;
  }

  return false;
}
//
//////////////////////////////////////////////////////////////////////////////

  };
};

