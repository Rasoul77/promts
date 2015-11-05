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

#ifndef _GEOMETRY_CENTROID_HPP_
#define _GEOMETRY_CENTROID_HPP_

#include "rasoul_geometry_pkg/GeometryDataTypes.hpp"

#include <Eigen/Eigen>

namespace rasoul{
  namespace geometry{

using namespace Eigen;

template <typename T>
Matrix<T, 3, 1>
TetrahedronCentroid(const Matrix<T, 3, 1>& O, const Matrix<T, 3, 1>& A,
                    const Matrix<T, 3, 1>& B, const Matrix<T, 3, 1>& C)
{
  DoubleFloatOnly<T> test;

  Matrix<T,3,1> a = A - O;
  Matrix<T,3,1> b = B - O;
  Matrix<T,3,1> c = C - O;

  return O + (a+b+c)/4;
}

template<typename T>
void
PolygonCentroid3D
(
  const std::vector< Matrix<T,3,1> >& v,
  Matrix<T,3,1>& C
)
{
  size_t N = v.size();
  if(N == 1){C = v[0]; return;}

  if(N == 2){C = (v[0]+v[1])/2.0f; return;}

  Matrix<T,3,1> n, e1, e2, origin, vv0;
  origin = v[0];
  e1 = v[1] - v[0];
  e2 = v[2] - v[0];
  n = e1.cross(e2);
  e2 = n.cross(e1);
  e1.normalize();
  e2.normalize();

  std::vector< Matrix<T,3,1> > v2d;
  v2d.resize(N);
  for(size_t i=0; i<N; i++)
  {
    vv0 = v[i] - origin;
    v2d[i].x() = vv0.dot(e1);
    v2d[i].y() = vv0.dot(e2);
  }

  float Cx = 0, Cy = 0, A = 0;
  for(size_t i=0; i<N-1; i++)
  {
    Cx += (v2d[i].x() + v2d[i+1].x())*(v2d[i].x()*v2d[i+1].y() - v2d[i+1].x()*v2d[i].y());
    Cy += (v2d[i].y() + v2d[i+1].y())*(v2d[i].x()*v2d[i+1].y() - v2d[i+1].x()*v2d[i].y());
    A  += (v2d[i].x()*v2d[i+1].y() - v2d[i+1].x()*v2d[i].y());
  }
  A /= 2.0f;
  Cx /= (6.0f*A);
  Cy /= (6.0f*A);

  C = origin + Cx*e1 + Cy*e2;
}

template<typename T>
Matrix<T,3,1>
PolygonCentroid3D
(
  const std::vector< Matrix<T,3,1> >& v
)
{
  Matrix<T,3,1> C;
  PolygonCentroid3D(v, C);
  return C;
}

  }; // namespace geometry
}; // namespace rasoul
#endif


