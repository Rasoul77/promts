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

#ifndef _COLLISION_DETECTION_SAT_HPP_
#define _COLLISION_DETECTION_SAT_HPP_

#include "rasoul_geometry_pkg/GeometryDataTypes.hpp"
#include "rasoul_geometry_pkg/GeometryPolyhedra.hpp"

namespace rasoul{
  namespace collision{

    using namespace Eigen;

    template<typename T>
    bool
    ComputeOverlappingValue
    (
      const geometry::CGeometryPolyhedra<T>& ph1,
      const geometry::CGeometryPolyhedra<T>& ph2,
      const Matrix<T,3,1>& axis,
      Matrix<T,3,1>& tv,
      T& ov
    )
    {
      T minim1 = ph1.V[0].dot(axis);
      T maxim1 = minim1;
      for(size_t i=1; i<ph1.V.size(); i++)
      {
        T temp = ph1.V[i].dot(axis);
        if(temp < minim1) minim1 = temp;
        else if(temp > maxim1) maxim1 = temp;
      }

      T minim2 = ph2.V[0].dot(axis);
      T maxim2 = minim2;
      for(size_t i=1; i<ph2.V.size(); i++)
      {
        T temp = ph2.V[i].dot(axis);
        if(temp < minim2) minim2 = temp;
        else if(temp > maxim2) maxim2 = temp;
      }

      ov = T(0);
      if(maxim1 <= minim2 || maxim2 <= minim1) return(false);
      if(minim1<minim2 && minim2<maxim1 && maxim1<maxim2)
      {
        ov = maxim1-minim2;
        tv = -axis;
      }
      else if(minim2<minim1 && minim1<maxim2 && maxim2<maxim1)
      {
        ov = maxim2-minim1;
        tv = axis;
      }
      else if(minim1<=minim2 && maxim2<=maxim1)
      {
        T a = maxim2-minim1;
        T b = maxim1-minim2;
        if(a<b) {ov = a; tv = axis;} else {ov = b; tv =-axis;}
      }
      else if(minim2<=minim1 && maxim1<=maxim2)
      {
        T a = maxim2-minim1;
        T b = maxim1-minim2;
        if(a<b) {ov = a; tv = axis;} else {ov = b; tv =-axis;}
      }
      else{
        std::cout << "<" << minim1 << ", " << maxim1 << "> , <" << minim2 << ", " << maxim2 << ">" << std::endl;
      }
      if(ov<0) std::cout << "<" << minim1 << ", " << maxim1 << "> , <" << minim2 << ", " << maxim2 << ">" << std::endl;
      return(true);
    }

    template<typename T>
    bool
    SAT3D
    (
      geometry::CGeometryPolyhedra<T>& ph1,
      const Transform<T,3,Affine>& pose1,
      geometry::CGeometryPolyhedra<T>& ph2,
      const Transform<T,3,Affine>& pose2,
      Matrix<T,3,1>& dop_vec,
      T& dop
    )
    {
      T ov;
      Matrix<T,3,1> tv;
      dop = std::numeric_limits<T>::infinity();
      ph1.TransformShape(pose1);
      ph2.TransformShape(pose2);

      for(size_t i=0; i<ph1.uF.size(); i++)
      {
        Matrix<T,3,1> axis = ph1.F[ ph1.uF[i] ].endp - ph1.F[ ph1.uF[i] ].cm;
        if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
        {
          if(ov<dop)
          {
            dop = ov;
            dop_vec = tv;
          }
        }
        else{
          return(false);
        }
      }

      for(size_t i=0; i<ph2.uF.size(); i++)
      {
        Matrix<T,3,1> axis = ph2.F[ ph2.uF[i] ].endp - ph2.F[ ph2.uF[i] ].cm;
        if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
        {
          if(ov<dop)
          {
            dop = ov;
            dop_vec = tv;
          }
        }
        else{
          return(false);
        }
      }

      for(size_t i=0; i<ph1.uE.size(); i++)
      {
        for(size_t j=0; j<ph2.uE.size(); j++)
        {
          Matrix<T,3,1> u1 = ph1.E[ ph1.uE[i] ].endp - ph1.V[ ph1.E[ ph1.uE[i] ].i ];
          Matrix<T,3,1> u2 = ph2.E[ ph2.uE[j] ].endp - ph2.V[ ph2.E[ ph2.uE[j] ].i ];
          Matrix<T,3,1> axis = u1.cross(u2);
          axis.normalize();
          if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
          {
            if(ov<dop)
            {
              dop = ov;
              dop_vec = tv;
            }
          }
          else{
            return(false);
          }
        }
      }
      dop_vec.normalize();
      return(true);
    }

    template<typename T>
    bool
    AllSAT3D
    (
      geometry::CGeometryPolyhedra<T>& ph1,
      const Transform<T,3,Affine>& pose1,
      geometry::CGeometryPolyhedra<T>& ph2,
      const Transform<T,3,Affine>& pose2,
      std::vector< Matrix<T,3,1> >& dop_vecs,
      std::vector< T >& dops
    )
    {
      dops.clear();
      dop_vecs.clear();

      T ov;
      Matrix<T,3,1> tv;

      ph1.TransformShape(pose1);
      ph2.TransformShape(pose2);

      for(size_t i=0; i<ph1.uF.size(); i++)
      {
        Matrix<T,3,1> axis = ph1.F[ ph1.uF[i] ].endp - ph1.F[ ph1.uF[i] ].cm;
        if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
        {
          dops.push_back(ov);
          dop_vecs.push_back(tv);
        }
        else{
          dops.clear();
          dop_vecs.clear();
          return(false);
        }
      }

      for(size_t i=0; i<ph2.uF.size(); i++)
      {
        Matrix<T,3,1> axis = ph2.F[ ph2.uF[i] ].endp - ph2.F[ ph2.uF[i] ].cm;
        if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
        {
          dops.push_back(ov);
          dop_vecs.push_back(tv);
        }
        else{
          dops.clear();
          dop_vecs.clear();
          return(false);
        }
      }

      for(size_t i=0; i<ph1.uE.size(); i++)
      {
        for(size_t j=0; j<ph2.uE.size(); j++)
        {
          Matrix<T,3,1> u1 = ph1.E[ ph1.uE[i] ].endp - ph1.V[ ph1.E[ ph1.uE[i] ].i ];
          Matrix<T,3,1> u2 = ph2.E[ ph2.uE[j] ].endp - ph2.V[ ph2.E[ ph2.uE[j] ].i ];
          Matrix<T,3,1> axis = u1.cross(u2);
          axis.normalize();
          if(ComputeOverlappingValue(ph1, ph2, axis, tv, ov))
          {
            dops.push_back(ov);
            dop_vecs.push_back(tv);
          }
          else{
            dops.clear();
            dop_vecs.clear();
            return(false);
          }
        }
      }

      return(true);
    }
  }; // namespace geometry
}; // namespace rasoul
#endif

