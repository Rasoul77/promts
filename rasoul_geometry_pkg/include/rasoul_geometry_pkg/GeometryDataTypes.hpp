/*
* Copyright (c) 2015 Center for Applied Autonomous Sensor Systems (AASS),
* Orebro University, Sweden
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are strictly limited to non-commercial academic purposes and are only permitted
* provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Orebro University Sweden nor the name Center for Applied
*       Autonomous Sensor Systems nor the names of its contributors may be used to
*       endorse or promote products derived from this software without specific prior
*       written permission.
*
* If you are interested in using this code for a commercial or non-academic purpose,
* please contact us.
*
* THIS SOFTWARE IS PROVIDED BY Mobile Robotics & Olfaction Group at AASS ``AS IS''
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL Robotics & Olfaction Group at AASS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Contact:
*    Rasoul Mojtahedzadeh,
*                         rasoul.mojtahedzadeh@oru.se
*                         mojtahedzadeh@gmail.com
*	   Achim J. Lilienthal,
*                         achim.lilienthal@oru.se
*
* Paper Mail:
*
*   Prof. Achim J. Lilienthal
*   Orebro University
*   Teknikhuset, T1222
*   70281 Orebro
*   Sweden
*/

#ifndef _GEOMETRY_COMMON_TYPES_H_
#define _GEOMETRY_COMMON_TYPES_H_

#include <vector>

#include <Eigen/Eigen>

namespace rasoul{
  namespace geometry{

using namespace Eigen;

template<typename T>
struct AABBox
{
  T minX;
  T minY;
  T minZ;
  T maxX;
  T maxY;
  T maxZ;

  AABBox() : minX(T(0)), minY(0.0), minZ(0.0), maxX(1.0), maxY(1.0), maxZ(1.0) {}

  AABBox& operator=(const AABBox &rhs)
  {
    minX = rhs.minX;
    minY = rhs.minY;
    minZ = rhs.minZ;
    maxX = rhs.maxX;
    maxY = rhs.maxY;
    maxZ = rhs.maxZ;
    return *this;
  }

  bool isConsistant() const
  {
    if(minX <= maxX && minY <= maxY && minZ <= maxZ)
      return(true);

    return(false);
  }

  void getLengths(T &Lx, T &Ly, T &Lz) const
  {
    Lx = maxX - minX;
    Ly = maxY - minY;
    Lz = maxZ - minZ;
  }

  T getRadius() const
  {
    T Lx = (maxX - minX)/2.0f;
    T Ly = (maxY - minY)/2.0f;
    T Lz = (maxZ - minZ)/2.0f;
    return( sqrt( Lx*Lx + Ly*Ly + Lz*Lz ) );
  }
};

template<typename T>
struct Edge
{
  unsigned int i;
  unsigned int j;
  Matrix<T,3,1> u;
  Matrix<T,3,1> endp;
};

template<typename T>
struct Face
{
  std::vector<unsigned int> Idx;
  Matrix<T,3,1> normal;
  Matrix<T,3,1> cm;
  Matrix<T,3,1> endp;
};

template <class T>
class DoubleFloatOnly
{
private:
    void ValidateType( double &d ) const {}
    void ValidateType( float  &f ) const {}

public:
    DoubleFloatOnly()
    {
       T valid;
       ValidateType( valid );
    };
};

  }; // namespace geometry
}; // namespace rasoul
#endif
