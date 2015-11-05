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

#ifndef _GEOMETRY_MISC_HPP_
#define _GEOMETRY_MISC_HPP_

#include <Eigen/Eigen>

namespace rasoul{
  namespace geometry{

using namespace Eigen;

template <typename T> int sgn(T val) {
    if( val <= -std::numeric_limits<T>::epsilon() ) return(-1);
    if( val >= std::numeric_limits<T>::epsilon() ) return(1);
    return(0);
}

template <typename T> int sgn(T val, T mul) {
    if( val <= -mul*std::numeric_limits<T>::epsilon() ) return(-1);
    if( val >= mul*std::numeric_limits<T>::epsilon() ) return(1);
    return(0);
}

template<typename T>
void
OrderVertices(std::vector< Matrix<T,3,1> >& v, std::vector<unsigned int>& I)
{
  unsigned int N = v.size();
  if(N < 4) return;

  Matrix<T,3,1> avg = Matrix<T,3,1>::Zero();
  for(unsigned int i=0; i<N; i++)
    avg += v[i];
  avg /= (T) N;

  Matrix<T,3,1> vec0 = v[0] - avg;
  vec0.normalize();

  T minAngle = std::numeric_limits<T>::infinity();
  unsigned int minIndex = 0;
  for(size_t i=1; i<N; i++)
  {
    Matrix<T,3,1> veci = v[i] - avg;
    veci.normalize();
    T angle = acos(veci.dot(vec0));
    if(angle < minAngle)
    {
      minAngle = angle;
      minIndex = i;
    }
  }

  Matrix<T,3,1> temp = v[1];
  v[1] = v[minIndex];
  v[minIndex] = temp;
  unsigned int tmp = I[1];
  I[1] = I[minIndex];
  I[minIndex] = tmp;


  temp = v[1] - avg;
  Matrix<T,3,1> n0 = vec0.cross(temp);

  for(unsigned int cur = 1; cur < N-1; cur++)
  {
    Matrix<T,3,1> refv = v[cur] - avg;
    refv.normalize();
    T minAngle = std::numeric_limits<T>::infinity();
    unsigned int minIndex = cur;
    for(size_t i=cur+1; i<N; i++)
    {
      Matrix<T,3,1> veci = v[i] - avg;
      Matrix<T,3,1> cr = refv.cross(veci);
      if( geometry::sgn<T>(n0.dot(cr)) < 0 ) continue;
      veci.normalize();
      T angle = acos(veci.dot(refv));
      if(angle < minAngle)
      {
        minAngle = angle;
        minIndex = i;
      }
    }
    Matrix<T,3,1> temp = v[cur+1];
    v[cur+1] = v[minIndex];
    v[minIndex] = temp;
    unsigned int tmp = I[cur+1];
    I[cur+1] = I[minIndex];
    I[minIndex] = tmp;
  }
}

  }; // namespace geometry
}; // namespace rasoul
#endif
