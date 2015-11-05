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
#ifndef _GEOMETRY_TRANSFORM_HPP_
#define _GEOMETRY_TRANSFORM_HPP_

#include "rasoul_geometry_pkg/GeometryDataTypes.hpp"
#include <Eigen/Eigen>

namespace rasoul{
  namespace geometry{

template <typename T>
void
TraQua2OpenGL(T *t, T *q, T *rotmat)
{
  DoubleFloatOnly<T> test;

  T n = 1.0/sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] *= n;
  q[1] *= n;
  q[2] *= n;
  q[3] *= n;

  rotmat[0] = 1.0-2.0*q[1]*q[1]-2.0*q[2]*q[2]; rotmat[4] = 2.0*q[0]*q[1]-2.0*q[2]*q[3];     rotmat[8]  = 2.0*q[0]*q[2]+2.0*q[1]*q[3];     rotmat[12] = t[0];
  rotmat[1] = 2.0*q[0]*q[1]+2.0*q[2]*q[3];     rotmat[5] = 1.0-2.0*q[0]*q[0]-2.0*q[2]*q[2]; rotmat[9]  = 2.0*q[1]*q[2]-2.0*q[0]*q[3];     rotmat[13] = t[1];
  rotmat[2] = 2.0*q[0]*q[2]-2.0*q[1]*q[3];     rotmat[6] = 2.0*q[1]*q[2]+2.0*q[0]*q[3];     rotmat[10] = 1.0-2.0*q[0]*q[0]-2.0*q[1]*q[1]; rotmat[14] = t[2];
  rotmat[3] = 0.0;                             rotmat[7] = 0.0;                             rotmat[11] = 0.0;                             rotmat[15] = 1.0;
}

template <typename T>
T
Reminder(T X, T Y)
{
  return X-int(X/Y)*Y;
}

template <typename T>
T
Omega(T angle)
{
  DoubleFloatOnly<T> test;
  if(2.0*M_PI<=angle){ angle = Reminder(angle,2.0*M_PI); }
  else
  if(angle<0){ angle = Reminder(angle,2.0*M_PI)+2.0*M_PI; }

  return(angle);
}

template <typename T>
void
OpenGL2Eulers(T *opengl, T *eulers)
{
  DoubleFloatOnly<T> test;
  eulers[0] = atan2(opengl[6], opengl[10]);
  eulers[1] = atan2(-opengl[2], sqrt(opengl[6]*opengl[6]+opengl[10]*opengl[10]));
  eulers[2] = atan2(opengl[1], opengl[0]);
}

template <typename T>
void
OpenGL2Eulers(std::vector<T>& opengl, T *eulers)
{
  DoubleFloatOnly<T> test;
  eulers[0] = atan2(opengl[6], opengl[10]);
  eulers[1] = atan2(-opengl[2], sqrt(opengl[6]*opengl[6]+opengl[10]*opengl[10]));
  eulers[2] = atan2(opengl[1], opengl[0]);
}

template <typename T>
void
RotMatrix2Eulers(const Matrix3f& R, Vector3f& euler)
{
  DoubleFloatOnly<T> test;

  euler.x() = atan2(R(2,1), R(2,2));
  euler.y() = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  euler.z() = atan2(R(1,0), R(0,0));
}

template <typename T>
void
Quaternion2Euler(T *q, T& roll_x, T& pitch_y, T& yaw_z)
{
  DoubleFloatOnly<T> test;

  T w2 = q[3]*q[3];
  T x2 = q[0]*q[0];
  T y2 = q[1]*q[1];
  T z2 = q[2]*q[2];
  T unitLength = w2+x2+y2+z2;
  T abcd = q[3]*q[0]+q[1]*q[2];
  T epsil = std::numeric_limits<T>::epsilon();
  T pi = 4*atan(1);

  if (abcd > (0.5-epsil)*unitLength)
  {
      yaw_z = 2 * atan2(q[1], q[3]);
      pitch_y = pi;
      roll_x = 0;
  }
  else if (abcd < (-0.5+epsil)*unitLength)
  {
      yaw_z = -2 * ::atan2(q[1], q[3]);
      pitch_y = -pi;
      roll_x = 0;
  }
  else
  {
      const double adbc = q[3]*q[2] - q[0]*q[1];
      const double acbd = q[3]*q[1] - q[0]*q[2];
      yaw_z = ::atan2(2*adbc, 1 - 2*(z2+x2));
      pitch_y = ::asin(2*abcd/unitLength);
      roll_x = ::atan2(2*acbd, 1 - 2*(y2+x2));
  }
}

template <typename T>
void
Euler2Quaternion(T roll_x, T pitch_y, T yaw_z, T *q)
{
  DoubleFloatOnly<T> test;

  T phi2 = roll_x/2.0;
  T tet2 = pitch_y/2.0;
  T saw2 = yaw_z/2.0;
  q[3] = cos(phi2)*cos(tet2)*cos(saw2) + sin(phi2)*sin(tet2)*sin(saw2);
  q[0] = sin(phi2)*cos(tet2)*cos(saw2) - cos(phi2)*sin(tet2)*sin(saw2);
  q[1] = cos(phi2)*sin(tet2)*cos(saw2) + sin(phi2)*cos(tet2)*sin(saw2);
  q[2] = cos(phi2)*cos(tet2)*sin(saw2) - sin(phi2)*sin(tet2)*cos(saw2);
}

template <typename T>
void
AffineTransform(T* p, T* trf)
{
  DoubleFloatOnly<T> test;

  T q[3];
  q[0] = trf[0] * p[0] + trf[4] * p[1] + trf[8] * p[2] + trf[12];
  q[1] = trf[1] * p[0] + trf[5] * p[1] + trf[9] * p[2] + trf[13];
  q[2] = trf[2] * p[0] + trf[6] * p[1] + trf[10] * p[2] + trf[14];
  p[0] = q[0]; p[1] = q[1]; p[2] = q[2];
}

inline double
Radian2Degree(double radian)
{
  return(radian*180.0/M_PI);
}

inline double
Degree2Radian(double degree)
{
  return(degree*M_PI/180.0);
}

  }; // namespace geoetry
}; // namespace rasoul
#endif
