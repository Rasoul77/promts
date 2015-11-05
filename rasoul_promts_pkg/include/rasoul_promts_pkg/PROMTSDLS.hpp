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
#ifndef _PROMTS_DLS_HPP_
#define _PROMTS_DLS_HPP_

#include "rasoul_common_pkg/Misc.hpp"
#include "rasoul_visualizer_pkg/OpenGLRosCom.hpp"
#include "rasoul_common_pkg/TimeFunctions.hpp"
#include "rasoul_promts_pkg/SearchCommon.hpp"
#include "rasoul_geometry_pkg/GeometryBBox.hpp"
#include "rasoul_collision_detection_pkg/CollisionDetectionSAT.hpp"

#include <map>
#include <limits>

namespace rasoul
{
  namespace promts
  {
  using namespace Eigen;
  using namespace std;

  class DLSearchNode
  {
  public:

    PoseVector poses;
    Real total_cost_to_reach_this_node;
    DLSearchNode* parent;
    ShapeVector* shapesPtr;
    IDVector* IDPtr;
    std::vector<DLSearchNode*> child;

    DLSearchNode() { isAtGoal = false; total_cost_to_reach_this_node = 0; }
    DLSearchNode(
      PoseVector& in,
      Real c,
      DLSearchNode* p,
      ShapeVector* shPtr,
      IDVector* idPtr
    )
    {
      isAtGoal = false;
      poses = in;
      total_cost_to_reach_this_node = c;
      parent = p;
      shapesPtr = shPtr;
      IDPtr = idPtr;
    }
    ~DLSearchNode()
    {
      for(size_t i=0; i<child.size(); i++)
        delete child[i];
    }

    bool isGoal()
    {
      if(isAtGoal) return(true);
      return(false);
    }

    void Compute()
    {
      std::vector< geometry::AABBox<Real> > aabb(shapesPtr->size());
      Real total_inter_penetration = 0.0f;
      // revert all the shapes to their original poses
      for(size_t i=0; i<shapesPtr->size(); i++)
        shapesPtr->at(i).RevertPose();
      // transform all the shapes to the poses
      for(size_t i=0; i<shapesPtr->size(); i++)
        shapesPtr->at(i).TransformShape(poses[i]);
      // compute AABBs
      for(size_t i=0; i<shapesPtr->size(); i++)
      {
        geometry::bbComputePointsAABB(shapesPtr->at(i).V, aabb[i]);
        geometry::bbExtendAABB(aabb[i], (Real) 0.01);
      }
      // compute DOPs
      PoseVector newPoses;
      for(size_t i=0; i<shapesPtr->size(); i++)
      {
        for(size_t j=i+1; j<shapesPtr->size(); j++)
        {
          // if two objects are fixed, skip!
          if(IDPtr->at(i) <= 0 && IDPtr->at(j) <= 0) continue;
          // if two objects AABBs collide, compute their possible DOP
          if(geometry::bbCheckTwoAABBCollision(aabb[i], aabb[j]))
          {
            Matrix<Real,3,1> dop_v;
            Real dop;
            if(collision::SAT3D(shapesPtr->at(i), eTransform::Identity(),
                               shapesPtr->at(j), eTransform::Identity(),
                               dop_v, dop)
              )
            {
              dop *= 1.05;
              Matrix<Real,3,1> v(dop*dop_v.x(),dop*dop_v.y(),dop*dop_v.z());
              if(IDPtr->at(i) > 0)
              {
                newPoses = poses;
                newPoses[i].data()[TX] = poses[i].data()[TX]+v.x();
                newPoses[i].data()[TY] = poses[i].data()[TY]+v.y();
                newPoses[i].data()[TZ] = poses[i].data()[TZ]+v.z();
                if(not isVisited(newPoses))
                {
                  total_inter_penetration += dop;
                  DLSearchNode* newNodePtr = new DLSearchNode(newPoses, total_cost_to_reach_this_node+dop, this, shapesPtr, IDPtr);
                  child.push_back(newNodePtr);
                }
              }
              if(IDPtr->at(j) > 0)
              {
                newPoses = poses;
                newPoses[j].data()[TX] = poses[j].data()[TX]-v.x();
                newPoses[j].data()[TY] = poses[j].data()[TY]-v.y();
                newPoses[j].data()[TZ] = poses[j].data()[TZ]-v.z();
                if(not isVisited(newPoses))
                {
                  total_inter_penetration += dop;
                  DLSearchNode* newNodePtr = new DLSearchNode(newPoses, total_cost_to_reach_this_node+dop, this, shapesPtr, IDPtr);
                  child.push_back(newNodePtr);
                }
              }
            }
          }
        }
      }
      if(child.empty()) isAtGoal = true;
      else if(total_inter_penetration/child.size() < 0.005) isAtGoal = true;
    }

    void getSolution(std::vector<DLSearchNode*>& solution)
    {
      solution.clear();
      DLSearchNode* node = this;
      solution.push_back(node);
      while(node->parent!=NULL)
      {
        node = node->parent;
        solution.push_back(node);
      }
    }

    bool isVisited(PoseVector& newPoses)
    {
      DLSearchNode* theparent = parent;
      while(theparent)
      {
        Real sum = 0;
        for(size_t i=0; i<newPoses.size(); i++)
        {
          Real dx = newPoses[i].data()[TX]-theparent->poses[i].data()[TX];
          Real dy = newPoses[i].data()[TY]-theparent->poses[i].data()[TY];
          Real dz = newPoses[i].data()[TZ]-theparent->poses[i].data()[TZ];
          sum += dx*dx+dy*dy+dz*dz;
        }
        if(sum < 0.001) return(true);
        theparent = theparent->parent;
      }
      return(false);
    }

    Real GetTotalDOP()
    {
      return total_cost_to_reach_this_node;
    }

    void PrintNodeInfo(COpenGLRosCom* glNodePtr)
    {
      Matrix<Real,3,1> color(0.1f, 0.5f, 0.4f);
      for(unsigned int i=0; i<shapesPtr->size(); i++)
      {
        shapesPtr->at(i).RevertPose();
        for(size_t j=0; j<shapesPtr->at(i).F.size(); j++)
        {
          glNodePtr->LineWidth(1.0f);
          glNodePtr->AddColor3(color.x(), color.y(), color.z());
          glNodePtr->AddBegin("LINE_LOOP");
          for(size_t k=0; k<shapesPtr->at(i).F[j].Idx.size(); k++)
          {
            int idx = shapesPtr->at(i).F[j].Idx[k];
            Eigen::Matrix<Real,3,1> Vt = poses[i]*shapesPtr->at(i).V[ idx ];
            glNodePtr->AddVertex(Vt.x(), Vt.y(), Vt.z());
          }
          glNodePtr->AddEnd();
        }
      }
      glNodePtr->SendCMDbuffer();
      ros::spinOnce();
    }

    private:
      bool  isAtGoal;
  };

  CSolution<DLSearchNode> RecursiveDLS(DLSearchNode* nodePtr, int limit, ProblemDataStruct& pds)
  {
    pds.m_iterations++;

    if(pds.m_iterations > pds.m_maxNodes) {return CSolution<DLSearchNode>(CSolution<DLSearchNode>::FAILURE);}
    nodePtr->Compute();
    if(pds.m_glNodePtr != NULL)
    {
      pds.m_glNodePtr->ResetCMDbuffer();
      nodePtr->PrintNodeInfo(pds.m_glNodePtr);
    }

    if(nodePtr->isGoal())
    {
      CSolution<DLSearchNode> S(CSolution<DLSearchNode>::SUCCESS);
      nodePtr->getSolution(S.m_solution);
      return S;
    }
    else if(limit==0) return CSolution<DLSearchNode>(CSolution<DLSearchNode>::CUTOFF);
    else{
      bool cutoffOccured = false;
      for(size_t i=0; i<nodePtr->child.size(); i++)
      {
        CSolution<DLSearchNode> result = RecursiveDLS(nodePtr->child[i], limit-1, pds);
        if(result.isCutoff()) cutoffOccured = true;
        else if(not result.isFailure()) return result;
        if(result.isFailure()) return CSolution<DLSearchNode>(CSolution<DLSearchNode>::FAILURE);
      }
      if(cutoffOccured) return CSolution<DLSearchNode>(CSolution<DLSearchNode>::CUTOFF); else return CSolution<DLSearchNode>(CSolution<DLSearchNode>::FAILURE);
    }
  }

  CSolution<DLSearchNode> DepthLimitedSearch(DLSearchNode* initNodePtr, int limit, ProblemDataStruct& pds)
  {
    return RecursiveDLS(initNodePtr, limit, pds);
  }

  bool
  refinePoses_DLSearch
  (
    ShapeVector& shapes, // input: Shapes
    PoseVector& poses, // input: Poses
    IDVector& ID, // input: IDs
    PoseVector& refined_poses, // output: Refined poses
    ProblemDataStruct& pds
  )
  {
    refined_poses = poses;
    Matrix<Real,3,1> color(0.3f, 0.3f, 0.9f);

    DLSearchNode initNode(poses, 0, NULL, &shapes, &ID);

    common::timestamp_t t0, t1;
    cerr << "Depth limited search started...\n";
    t0 = common::get_timestamp();
    CSolution<DLSearchNode> S = DepthLimitedSearch(&initNode, pds.m_maxNodes, pds);
    t1 = common::get_timestamp();
    double dt = (double) (t1 - t0) / 1000000.0L;
    cerr << "Depth limited search finished in " << dt << " seconds\n";

		if( S.isSuccess() )
		{
      refined_poses = S.m_solution[0]->poses;
      pds.m_steps = S.m_solution.size();
      pds.m_exeTime = dt;
			cerr << "Depth limited search found a goal state in "<<S.m_solution.size()<<" steps\n";
			if(pds.m_glNodePtr != NULL)
			{
        for(int i=S.m_solution.size()-1; i>=0; i--)
        {
          pds.m_glNodePtr->ResetCMDbuffer();
          if(i==0) S.m_solution[i]->PrintNodeInfo(pds.m_glNodePtr);
          else S.m_solution[i]->PrintNodeInfo(pds.m_glNodePtr);
          if(pds.m_visualize_enter) {std::cerr << "[" << S.m_solution.size()-i << "]: Press <Enter>...";  getchar();}
          else common::wait(0.1);
        }
      }
		}
		else
		{
			cerr << "Depth limited search terminated. Did not find goal state\n";
			return(false);
		}

    return(true);
  }

  }; // namespace promts
}; // namespace rasoul
#endif


