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

#ifndef _GEOMETRY_POLYHEDRA_HPP_
#define _GEOMETRY_POLYHEDRA_HPP_

#include "rasoul_geometry_pkg/GeometryDataTypes.hpp"
#include "rasoul_geometry_pkg/GeometryMisc.hpp"
#include "rasoul_geometry_pkg/GeometryVolume.hpp"
#include "rasoul_geometry_pkg/GeometryCentroid.hpp"

#ifdef PROMTS_USE_BULLET
#include <LinearMath/btConvexHullComputer.h>
#include <LinearMath/btTransform.h>
#endif

#include <Eigen/Eigen>

namespace rasoul{
  namespace geometry{

  using namespace Eigen;

  template<typename T>
  class CGeometryPolyhedra
  {
    public:
      struct VecPoints
      {
        std::vector< Matrix<T,3,1> > p;
      };

      enum method_t {METHOD_BRUTE_FORDE=0, METHOD_BULLET};
      //======================================================================
      // Constructor
      //======================================================================
      CGeometryPolyhedra() : epsil(1e-3) {}

      bool
      Compute(const std::vector< Matrix<T,3,1> >& egV, method_t method_type = METHOD_BRUTE_FORDE, bool flag_check_euler = true)
      {
        ////////////////////////////////////////////////////////////////////////
        // clear all data
        ClearAll();
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Test for error
        if(egV.size() < 4) return(false);
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Fill in V
        V = egV;
        size_t nV = egV.size();
        //
        ////////////////////////////////////////////////////////////////////////

#ifdef PROMTS_USE_BULLET
        ////////////////////////////////////////////////////////////////////////
        // Compute 3D convex hull - BULLET Implementation
        if(method_type == METHOD_BULLET)
        {
          btAlignedObjectArray<btVector3> btV;
          btV.resize(egV.size());
          for(size_t i=0; i<egV.size(); i++) btV[i] = btVector3(egV[i].x(), egV[i].y(), egV[i].z());
          convexUtil.compute(&btV[0].getX(), sizeof(btVector3), btV.size(), 0, 0);
        }
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Remove edges split a polygon face into triangles - BULLET
        if(method_type == METHOD_BULLET)
        {
          std::vector<VecPoints> f3p;
          for(int f=0;f<convexUtil.faces.size();f++)
          {
            int face = convexUtil.faces[f];
            const btConvexHullComputer::Edge*  firstEdge = &convexUtil.edges[face];
            const btConvexHullComputer::Edge*  edge = firstEdge;
            VecPoints tp;
            tp.p.resize(3);
            for(int i=0; i<3; i++)
            {
              int src = edge->getSourceVertex();
              tp.p[i] = Matrix<T,3,1>(convexUtil.vertices[src].x(), convexUtil.vertices[src].y(), convexUtil.vertices[src].z());
              edge = edge->getNextEdgeOfFace();
            }
            f3p.push_back(tp);
          }
        }
        //
        ////////////////////////////////////////////////////////////////////////
#endif

        ////////////////////////////////////////////////////////////////////////
        // Compute 3D convex hull - BRUTE FORCE
        if(method_type == METHOD_BRUTE_FORDE)
        {
          std::vector<bool> cnr(nV);
          std::vector<unsigned int> pIdx;
          std::fill(cnr.begin() + nV - 3, cnr.end(), true);
          do
          {
            // Selecting 3 vertices from nv vertices
            for (unsigned int i = 0; i < nV; ++i)
            {
              if (cnr[i])
              {
                pIdx.push_back(i);
                if(pIdx.size() == 3) break;
              }
            }

            // Compute the distance of the other points to the plane that is identified by the three points
            Matrix<T,3,1> n = (V[pIdx[0]]-V[pIdx[1]]).cross(V[pIdx[2]]-V[pIdx[1]]);
            n.normalize();
            unsigned int neg_counter(0), pos_counter(0);
            std::vector<unsigned int> zeros;
            for(unsigned int i=0; i<nV; i++)
            {
              if(i==pIdx[0] || i==pIdx[1] || i==pIdx[2]) continue;
              T d = n.dot(V[i]-V[pIdx[1]]);
              if( d >= (T) 1e-3 ) pos_counter++;
              else if( d <= (T) -1e-3 ) neg_counter++;
              else zeros.push_back(i);
              if(pos_counter != 0 && neg_counter != 0) break;
            }

            // Test if all the other point remain on or in one side of the palne?
            if(pos_counter == 0 || neg_counter == 0) // THIS IS A FACE!
            {
              // First check if the plane is already identified by an face
              bool found = false;
              for(unsigned int i=0; i<F.size(); i++)
              {
                bool found0 = false, found1 = false, found2 = false;
                for(unsigned int j=0; j<F[i].Idx.size(); j++)
                  if(F[i].Idx[j] == pIdx[0]) found0 = true;
                  else if(F[i].Idx[j] == pIdx[1]) found1 = true;
                  else if(F[i].Idx[j] == pIdx[2]) found2 = true;
                if(found0 && found1 && found2) {found = true; break;}
              }
              // If the plane is not identified by so far detected faces, then add this new face
              if(not found)
              {
                pIdx.insert(pIdx.end(), zeros.begin(), zeros.end());
                std::vector< Matrix<T,3,1> > fv(pIdx.size());
                for(unsigned int i=0; i<fv.size(); i++) fv[i] = V[pIdx[i]];
                OrderVertices(fv, pIdx);
                Face<T> face;
                face.Idx = pIdx;
                F.push_back(face);
              }
            }
            pIdx.clear();
          } while (std::next_permutation(cnr.begin(), cnr.end()));
          // Fill in the center of mass, normal and end point of each face
          for(size_t f=0; f<F.size(); f++)
          {
            // Check for error
            if(F[f].Idx.size() < 3) {ClearAll(); return(false);}
            // Compute center of mass of the f-th face
            std::vector< Matrix<T,3,1> > vf(F[f].Idx.size());
            for(size_t k=0; k<F[f].Idx.size(); k++) vf[k] = V[ F[f].Idx[k] ];
            F[f].cm = PolygonCentroid3D(vf);
            // Compute normal vector of f-th face
            Matrix<T,3,1> e1 = V[F[f].Idx[1]] - V[F[f].Idx[0]];
            Matrix<T,3,1> e2 = V[F[f].Idx[2]] - V[F[f].Idx[0]];
            Matrix<T,3,1> nf = e2.cross(e1);
            T magf = nf.stableNorm();
            for(size_t i=3; i<F[f].Idx.size(); i++)
            {
              e2 = V[F[f].Idx[i]] - V[F[f].Idx[0]];
              Matrix<T,3,1> n = e2.cross(e1);
              T mag = n.stableNorm();
              if(magf<mag){ magf = mag; nf = n;}
            }
            F[f].normal = nf.normalized();
            // Compute end point of f-th face
            F[f].endp = F[f].cm + F[f].normal;
          }
        }
        //
        ////////////////////////////////////////////////////////////////////////

#ifdef PROMTS_USE_BULLET
        ////////////////////////////////////////////////////////////////////////
        // Fill in F
        if(method_type == METHOD_BULLET)
        {
          F.resize(convexUtil.faces.size());
          for(int f=0;f<convexUtil.faces.size();f++)
          {
            int face = convexUtil.faces[f];
            const btConvexHullComputer::Edge*  firstEdge = &convexUtil.edges[face];
            const btConvexHullComputer::Edge*  edge = firstEdge;

            do{
              int src  = edge->getSourceVertex();
              int targ = edge->getTargetVertex();

              //std::cerr << f << ": src - target > " << src << " - " << targ << std::endl;

              F[f].Idx.push_back(src);
              edge = edge->getNextEdgeOfFace();

            }while (edge!=firstEdge);

            if(F[f].Idx.size() < 3) return(false);

            std::vector< Matrix<T,3,1> > vf(F[f].Idx.size());
            for(size_t k=0; k<F[f].Idx.size(); k++) vf[k] = V[ F[f].Idx[k] ];
            F[f].cm = PolygonCentroid3D(vf);

            Matrix<T,3,1> e1 = V[F[f].Idx[1]] - V[F[f].Idx[0]];
            Matrix<T,3,1> e2 = V[F[f].Idx[2]] - V[F[f].Idx[0]];
            Matrix<T,3,1> nf = e2.cross(e1);
            T magf = nf.stableNorm();
            for(size_t i=3; i<F[f].Idx.size(); i++)
            {
              e2 = V[F[f].Idx[i]] - V[F[f].Idx[0]];
              Matrix<T,3,1> n = e2.cross(e1);
              T mag = n.stableNorm();
              if(magf<mag){ magf = mag; nf = n;}
            }
            F[f].normal = nf.normalized();
            F[f].endp = F[f].cm + F[f].normal;
          }
        }
        //
        ////////////////////////////////////////////////////////////////////////
#endif

        ////////////////////////////////////////////////////////////////////////
        // Fill in E
        Edge<T> eTemp;
        size_t vN = F[0].Idx.size();

        for(size_t v=0; v<vN; v++)
        {
          eTemp.i = F[0].Idx[v];
          eTemp.j = F[0].Idx[(v+1)%vN];
          eTemp.u = V[eTemp.j]-V[eTemp.i];
          eTemp.u.normalize();
          eTemp.endp = V[eTemp.i] + eTemp.u;
          E.push_back(eTemp);
        }

        for(size_t f=1; f<F.size(); f++)
        {
          vN = F[f].Idx.size();
          for(size_t v=0; v<vN; v++)
          {
            eTemp.i = F[f].Idx[v];
            eTemp.j = F[f].Idx[(v+1)%vN];

            bool found = false;
            for(size_t e=0; e<E.size(); e++)
            {
              if( (eTemp.i == E[e].i && eTemp.j == E[e].j) || (eTemp.i == E[e].j && eTemp.j == E[e].i) )
              {
                found = true;
                break;
              }
            }
            if(not found)
            {
              eTemp.u = V[eTemp.j]-V[eTemp.i];
              eTemp.u.normalize();
              eTemp.endp = V[eTemp.i] + eTemp.u;
              E.push_back(eTemp);
            }
          }
        }
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Fill in uF (Unique Faces)
        uF.push_back(0);
        for(size_t i=1; i<F.size(); i++)
        {
          bool found = false;
          for(size_t j=0; j<uF.size(); j++)
          {
            if( fabs( (T)1.0 - F[i].normal.dot(F[uF[j]].normal) ) < epsil )
            {
              found = true;
              break;
            }
          }
          if(not found) uF.push_back(i);
        }
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Fill in uE (Unique Edges)
        uE.push_back(0);
        for(size_t i=1; i<E.size(); i++)
        {
          bool found = false;
          for(size_t j=0; j<uE.size(); j++)
          {
            if( fabs( (T)1.0 - E[i].u.dot(E[uE[j]].u) ) < epsil )
            {
              found = true;
              break;
            }
          }
          if(not found) uE.push_back(i);
        }
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Center of mass of the polyhedron
        C = Matrix<T,3,1>::Zero();
        if(F.empty()) {ClearAll(); return(false);}
        unsigned int vi = F[0].Idx[0];
        std::vector<unsigned int> ftri;
        for(unsigned int i=1; i<F.size(); i++)
        {
          bool found = false;
          for(unsigned int j=0; j<F[i].Idx.size(); j++)
          {
            if(F[i].Idx[j] == vi)
            {
              found = true;
              break;
            }
          }
          if(not found)
            ftri.push_back(i);
        }
        T total_vol = T(0);
        for(unsigned int i=0; i<ftri.size(); i++)
        {
          for(unsigned int j=1; j<F[ftri[i]].Idx.size()-1; j++)
          {
            T vol = T(0);
            Matrix<T,3,1> cm;
            cm  = TetrahedronCentroid(V[vi], V[F[ftri[i]].Idx[0]], V[F[ftri[i]].Idx[j]], V[F[ftri[i]].Idx[j+1]]);
            vol = TetrahedronVolume(V[vi], V[F[ftri[i]].Idx[0]], V[F[ftri[i]].Idx[j]], V[F[ftri[i]].Idx[j+1]]);
            C += vol*cm;
            total_vol += vol;
          }
        }
        C /= total_vol;
        volume = total_vol;
        //
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // make a back up
        V_ = V;
        E_ = E;
        F_ = F;
        C_ = C;
        //
        ////////////////////////////////////////////////////////////////////////

        return(true);
      }

      //======================================================================
      // ClearAll()
      //======================================================================
      void ClearAll()
      {
        V.clear();
        F.clear();
        E.clear();
        uF.clear();
        uE.clear();
        C = Matrix<T,3,1>::Zero();
      }

      //======================================================================
      // MoveAlongVector()
      //======================================================================
      void MoveAlongVector(const Matrix<T,3,1>& vec, T mag)
      {
        for(size_t i=0; i<V.size(); i++) V[i] = V[i]+mag*vec;

        for(size_t i=0; i<F.size(); i++) {F[i].cm = F[i].cm+mag*vec; F[i].endp = F[i].endp+mag*vec;}

        for(size_t i=0; i<F.size(); i++) {E[i].endp = E[i].endp+mag*vec;}

        C = C+mag*vec;
      }

      //======================================================================
      // TransformShape()
      //======================================================================
      void TransformShape(const Transform<T,3,Affine>& pose)
      {

        for(size_t i=0; i<V.size(); i++) V[i] = pose * V[i];

        for(size_t i=0; i<F.size(); i++) {F[i].cm = pose * F[i].cm; F[i].endp = pose * F[i].endp;}

        for(size_t i=0; i<F.size(); i++) {E[i].endp = pose * E[i].endp;}

        C = pose * C;
      }

      //======================================================================
      // RotateShape()
      //======================================================================
      void RotateShape(const Matrix<T,3,3>& rot)
      {

        for(size_t i=0; i<V.size(); i++) V[i] = rot * V[i];

        for(size_t i=0; i<F.size(); i++) {F[i].cm = rot * F[i].cm; F[i].endp = rot * F[i].endp;}

        for(size_t i=0; i<F.size(); i++) {E[i].endp = rot * E[i].endp;}

        C = rot * C;
      }

      //======================================================================
      // RevertPose()
      //======================================================================
      void RevertPose()
      {
        V = V_;
        E = E_;
        F = F_;
        C = C_;
      }

      std::vector< Matrix<T,3,1> > V; // Vertices of the polyhedron
      std::vector< Face<T> > F;       // Faces of the polyhedron
      std::vector< Edge<T> > E;       // Edges of the polyhedron
      std::vector<unsigned int> uF;  // Indices of unique faces (i.e., parallel faces are elimiated)
      std::vector<unsigned int> uE;  // Indices of unique edges (i.e., parallel edges are elimiated)
      Matrix<T,3,1> C;                  // Center of mass of the polyhedron
      T volume;                         // Volume of the polyhedron
    private:
      T epsil;
#ifdef PROMTS_USE_BULLET
      btConvexHullComputer convexUtil;
#endif
      std::vector< Matrix<T,3,1> > V_; // Original vertices of the polyhedron
      std::vector< Face<T> > F_;       // Original faces of the polyhedron
      std::vector< Edge<T> > E_;       // Original edges of the polyhedron
      Matrix<T,3,1> C_;                 // Original center of mass of the polyhedron
  };

  }; // namespace geometry
}; // namespace rasoul
#endif
