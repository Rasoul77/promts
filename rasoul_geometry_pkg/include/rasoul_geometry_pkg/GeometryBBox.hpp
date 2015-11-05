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

#ifndef _GEOMETRY_BBOX_HPP_
#define _GEOMETRY_BBOX_HPP_

#include "rasoul_geometry_pkg/GeometryDataTypes.hpp"

namespace rasoul{
  namespace geometry{

  using namespace Eigen;

  template<typename T>
  void
  bbComputePointsAABB(const std::vector< Matrix<T,3,1> >& points, AABBox<T>& aabb)
  {
    if(points.empty()) return;

    aabb.maxX = -std::numeric_limits<T>::infinity();
    aabb.maxY = -std::numeric_limits<T>::infinity();
    aabb.maxZ = -std::numeric_limits<T>::infinity();
    aabb.minX = std::numeric_limits<T>::infinity();
    aabb.minY = std::numeric_limits<T>::infinity();
    aabb.minZ = std::numeric_limits<T>::infinity();
    for(size_t i=0; i<points.size(); i++)
    {
      if(points[i].x() > aabb.maxX) aabb.maxX = points[i].x();
      if(points[i].y() > aabb.maxY) aabb.maxY = points[i].y();
      if(points[i].z() > aabb.maxZ) aabb.maxZ = points[i].z();
      if(points[i].x() < aabb.minX) aabb.minX = points[i].x();
      if(points[i].y() < aabb.minY) aabb.minY = points[i].y();
      if(points[i].z() < aabb.minZ) aabb.minZ = points[i].z();
    }
  }

  template<typename T>
  bool
  bbCheckTwoAABBCollision(const AABBox<T>& aabb0, const AABBox<T>& aabb1)
  {
    if(aabb0.minX > aabb1.maxX || aabb1.minX > aabb0.maxX ||
       aabb0.minY > aabb1.maxY || aabb1.minY > aabb0.maxY ||
       aabb0.minZ > aabb1.maxZ || aabb1.minZ > aabb0.maxZ) return(false);

    return(true);
  }

  template<typename T>
  void
  bbExtendAABB(AABBox<T>& aabb, T d)
  {
    aabb.maxX += d;
    aabb.maxY += d;
    aabb.maxZ += d;
    aabb.minX -= d;
    aabb.minY -= d;
    aabb.minZ -= d;
  }

  }; // namespace geometry
}; // namespace rasoul
#endif
