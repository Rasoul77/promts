#include <rasoul_common_pkg/RandFunctions.hpp>

namespace rasoul{
  namespace common{

void initrand()
{
    srand((unsigned)(time(0)));
}

//generates a psuedo-random integer between 0 and 32767
int randint()
{
    return rand();
}

//generates a psuedo-random integer between 0 and max
int randint(int max)
{
    return int(max*(double)rand()/(RAND_MAX+1.0));
}

//generates a psuedo-random integer between min and max
int randint(int min, int max)
{
    if (min>max)
    {
        return max+int((min-max+1)*(double)rand()/(RAND_MAX+1.0));
    }
    else
    {
        return min+int((max-min+1)*(double)rand()/(RAND_MAX+1.0));
    }
}

//generates a psuedo-random float between 0.0 and 0.999...
float randfloat()
{
    return rand()/(float(RAND_MAX)+1);
}

//generates a psuedo-random float between 0.0 and max
float randfloat(float max)
{
    return randfloat()*max;
}

//generates a psuedo-random float between min and max
float randfloat(float min, float max)
{
    if (min>max)
    {
        return randfloat()*(min-max)+max;
    }
    else
    {
        return randfloat()*(max-min)+min;
    }
}

//generates a psuedo-random double between 0.0 and 0.999...
double randdouble()
{
    return rand()/(double(RAND_MAX)+1);
}

//generates a psuedo-random double between 0.0 and max
double randdouble(double max)
{
    return randdouble()*max;
}

//generates a psuedo-random double between min and max
double randdouble(double min, double max)
{
    if (min>max)
    {
        return randdouble()*(min-max)+max;
    }
    else
    {
        return randdouble()*(max-min)+min;
    }
}

char* randstr(char* str, unsigned long length)
{
    //make we were passed a valid pointer
    if (!str)
    {
        return 0;
    }

    //make sure the string is supposed to contain something
    if (!length)
    {
        return 0;
    }

    //put random characters into the string, give both
    //upper and lower case numbers an equal chance at
    //being used
    unsigned long x;
    for (x=0;x<length-1;x++)
    {
        if (!randint(1))
        {
            str[x]=(char)randint(65,90);
        }
        else
        {
            str[x]=(char)randint(97,122);
        }
    }

    //null terminate the string
    str[x]=0;

    return str;
}

char* randstr(char* str, unsigned long length, unsigned long flags)
{
    //make we were passed a valid pointer
    if (!str)
    {
        return 0;
    }

    //make sure the string is supposed to contain something
    if (!length)
    {
        return 0;
    }

    //if none of the flags were used then we set flags to use just upper and lower case
    if ((!flags&0x00000001)&&(!flags&0x00000002)&&(!flags&0x00000004)&&
    (!flags&0x00000008)&&(!flags&0x00000010)&&(!flags&0x00000020)&&(!flags&0x00000040)
    &&(!flags&0x00000080)&&(!flags&0x00000100))
    {
        flags=RAND_STR_LOWER_CASE_LETTERS|RAND_STR_CAPITOL_LETTERS;
    }

    char t[9]={0};
    int i=0;

    //each pass of the loop the flags are checked and for each flag that is used
    //then a random character in the flags range is added to a temporary array of
    //characters, after all the flags are checked a random character from the
    //temporary array is chosen to be inserted into the string
    unsigned long x;
    for (x=0;x<length-1;x++)
    {
        i=0;

        if (flags&0x1)
        {
            t[i]=randint(65,90);
            i++;
        }

        if (flags&0x2)
        {
            t[i]=randint(97,122);
            i++;
        }

        if (flags&0x4)
        {
            t[i]=randint(48,57);
            i++;
        }

        if (flags&0x8)
        {
            t[i]=randint(32,47);
            i++;
        }

        if (flags&0x10)
        {
            t[i]=randint(58,64);
            i++;
        }

        if (flags&0x20)
        {
            t[i]=randint(91,96);
            i++;
        }

        if (flags&0x40)
        {
            t[i]=randint(123,126);
            i++;
        }

        if (flags&0x80)
        {
            if (!randint(32))
            {
                t[i]=127;
                i++;
            }
            else
            {
                t[i]=randint(1,31);
                i++;
            }
        }

        if (flags&0x100)
        {
            t[i]=randint(127,255);
            i++;
        }

        str[x]=t[randint(i-1)];
    }

    //null terminate the string
    str[x]=0;

    return str;
}

  };
};
