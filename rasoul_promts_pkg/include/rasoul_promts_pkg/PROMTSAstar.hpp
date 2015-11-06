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

#ifndef _PROMTS_ASTAR_HPP_
#define _PROMTS_ASTAR_HPP_

#include "rasoul_common_pkg/Misc.hpp"
#include "rasoul_visualizer_pkg/OpenGLRosCom.hpp"
#include "rasoul_common_pkg/TimeFunctions.hpp"
#include "rasoul_promts_pkg/SearchCommon.hpp"
#include "rasoul_geometry_pkg/GeometryBBox.hpp"
#include "rasoul_collision_detection_pkg/CollisionDetectionSAT.hpp"

#include <map>
#include <limits>
#include <algorithm>

namespace rasoul
{
  namespace promts
  {
  using namespace Eigen;
  using namespace std;

  class AstarNode
  {
  public:

    // parent pointer and children pointers
    AstarNode* m_parentPtr;
    std::vector<AstarNode*> m_childPtr;

    // node state
    PoseVector m_poses;

    // cost to reach "this" node from parent node
    float m_costToReachThisNodeFromParent;

    // helper pointers to shapes and id's of the objects for computation
    ShapeVector* m_shapesPtr;
    IDVector* m_IDPtr;

    // constructors
    AstarNode() { m_isNodeAGoal = false; m_costToReachThisNodeFromParent = 0; }
    AstarNode(
      PoseVector& poses,
      float costToReachThisNodeFromParent,
      AstarNode* parentPtr,
      ShapeVector* shapesPtr,
      IDVector* IDPtr
    )
    {
      m_isNodeAGoal = false;
      m_poses = poses;
      m_costToReachThisNodeFromParent = costToReachThisNodeFromParent;
      m_parentPtr = parentPtr;
      m_shapesPtr = shapesPtr;
      m_IDPtr = IDPtr;
    }

    // deconstructor
    ~AstarNode()
    {
      for(size_t i=0; i<m_childPtr.size(); i++)
        delete m_childPtr[i];
    }

    // get distance of "this" node to idx-th child node
    inline float getDistToChild( unsigned int idx )
    {
      if(idx<m_costFromThisToChild.size())
      {
        return(m_costFromThisToChild[idx]);
      }
      return(0);
    }

    // get heuristic cost to reach a goal state from "this" node
    inline float getHeuristicCostToGoal( AstarNode* goalNodePtr )
    {
      return(m_estimatedCostToGoal);
    }

    // is "this" node a goal state?
    inline bool isGoal( AstarNode* goalNodePtr )
    {
      if(m_isNodeAGoal) return(true);
      return(false);
    }

    // compute children and other neccessary computations
    inline void compute()
    {
      std::vector< geometry::AABBox<Real> > aabb(m_shapesPtr->size());
      m_estimatedCostToGoal = 0.0f;
      // revert all the shapes to their original poses
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        m_shapesPtr->at(i).RevertPose();
        m_shapesPtr->at(i).TransformShape(m_poses[i]);
      }
      // compute AABBs
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        geometry::bbComputePointsAABB(m_shapesPtr->at(i).V, aabb[i]);
        geometry::bbExtendAABB(aabb[i], (Real) 0.01);
      }
      // compute DOPs
      PoseVector newPoses;
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        for(size_t j=i+1; j<m_shapesPtr->size(); j++)
        {
          // if two objects are fixed, skip!
          if(m_IDPtr->at(i) <= 0 && m_IDPtr->at(j) <= 0) continue;
          // if two objects AABBs collide, compute their possible DOP
          if(geometry::bbCheckTwoAABBCollision(aabb[i], aabb[j]))
          {
            Matrix<Real,3,1> dop_v;
            Real dop;
            if(collision::SAT3D(m_shapesPtr->at(i), eTransform::Identity(),
                               m_shapesPtr->at(j), eTransform::Identity(),
                               dop_v, dop)
              )
            {
              dop *= 1.05;
              m_estimatedCostToGoal += dop;
              Matrix<Real,3,1> v(dop*dop_v.x(),dop*dop_v.y(),dop*dop_v.z());
              if(m_IDPtr->at(i) > 0)
              {
                newPoses = m_poses;
                newPoses[i].data()[TX] = m_poses[i].data()[TX]+v.x();
                newPoses[i].data()[TY] = m_poses[i].data()[TY]+v.y();
                newPoses[i].data()[TZ] = m_poses[i].data()[TZ]+v.z();
                if(not isVisited(newPoses))
                {
                  AstarNode* newNodePtr = new AstarNode(newPoses, dop, this, m_shapesPtr, m_IDPtr);
                  m_childPtr.push_back(newNodePtr);
                }
              }
              if(m_IDPtr->at(j) > 0)
              {
                newPoses = m_poses;
                newPoses[j].data()[TX] = m_poses[j].data()[TX]-v.x();
                newPoses[j].data()[TY] = m_poses[j].data()[TY]-v.y();
                newPoses[j].data()[TZ] = m_poses[j].data()[TZ]-v.z();
                if(not isVisited(newPoses))
                {
                  AstarNode* newNodePtr = new AstarNode(newPoses, dop, this, m_shapesPtr, m_IDPtr);
                  m_childPtr.push_back(newNodePtr);
                }
              }
            }
          }
        }
      }
      if(m_estimatedCostToGoal/m_childPtr.size() < 0.005) m_isNodeAGoal = true;
    }

    // return the solution
    inline void getSolution(std::vector<AstarNode*>& solution)
    {
      solution.clear();
      AstarNode* node = this;
      solution.push_back(node);
      while(node->m_parentPtr!=NULL)
      {
        node = node->m_parentPtr;
        solution.push_back(node);
      }
    }

    // checks if the set of newPoses has been already visited
    inline bool isVisited(PoseVector& newPoses)
    {
      AstarNode* theparent = m_parentPtr;
      while(theparent)
      {
        float sum = 0;
        for(size_t i=0; i<newPoses.size(); i++)
        {
          float dx = newPoses[i].data()[TX]-theparent->m_poses[i].data()[TX];
          float dy = newPoses[i].data()[TY]-theparent->m_poses[i].data()[TY];
          float dz = newPoses[i].data()[TZ]-theparent->m_poses[i].data()[TZ];
          sum += dx*dx+dy*dy+dz*dz;
        }
        if(sum < 0.001) return(true);
        theparent = theparent->m_parentPtr;
      }
      return(false);
    }

    // print some info about the node
    inline void printNodeInfo(COpenGLRosCom* glNodePtr)
    {
      if(glNodePtr == NULL) return;
      Matrix<float,3,1> color(0.1f, 0.5f, 0.4f);
      for(unsigned int i=0; i<m_shapesPtr->size(); i++)
      {
        m_shapesPtr->at(i).RevertPose();
        for(size_t j=0; j<m_shapesPtr->at(i).F.size(); j++)
        {
          glNodePtr->LineWidth(1.0f);
          glNodePtr->AddColor3(color.x(), color.y(), color.z());
          glNodePtr->AddBegin("LINE_LOOP");
          for(size_t k=0; k<m_shapesPtr->at(i).F[j].Idx.size(); k++)
          {
            int idx = m_shapesPtr->at(i).F[j].Idx[k];
            Eigen::Matrix<float,3,1> Vt = m_poses[i]*m_shapesPtr->at(i).V[ idx ];
            glNodePtr->AddVertex(Vt.x(), Vt.y(), Vt.z());
          }
          glNodePtr->AddEnd();
        }
      }
      glNodePtr->SendCMDbuffer();
      ros::spinOnce();
    }


  private:
    std::vector<float> m_costFromThisToChild;
    bool  m_isNodeAGoal;
    float m_estimatedCostToGoal;
  };

  template<typename CNodeT>
  inline CSolution<CNodeT> A_Star_Search(CNodeT* startNodePtr, CNodeT* goalNodePtr, ProblemDataStruct& pds)
  {
    std::vector<CNodeT*> closedSet; // The set of nodes already evaluated.
    std::vector<CNodeT*> openSet;   // The set of tentative nodes to be evaluated,
    openSet.push_back(startNodePtr);    // initially containing the start node
    std::map<CNodeT*, float> g_score; // Cost from start along best known path.
    g_score[startNodePtr] = 0; // initial node cost is zero
    std::map<CNodeT*, float> f_score; // Estimated total cost from start to goal through path y.

    startNodePtr->compute();
    f_score[startNodePtr] = g_score[startNodePtr] + startNodePtr->getHeuristicCostToGoal(goalNodePtr);

    while(not openSet.empty())
    {
      // give up if the number of expanded nodes are greater than some threshold
      if(openSet.size()+closedSet.size() > (unsigned) pds.m_maxNodes)
      {
        pds.m_iterations = openSet.size()+closedSet.size();
        CSolution<CNodeT> S(CSolution<CNodeT>::FAILURE);
        return S;
      }

      // current := the node in openset having the lowest f_score[] value
      size_t current_idx = 0;
      CNodeT* current = openSet[current_idx];
      Real lowest_f_score = f_score[openSet[current_idx]];
      for(size_t i=1; i<openSet.size(); i++)
        if(f_score[openSet[i]] < lowest_f_score)
        {
          lowest_f_score = f_score[openSet[i]];
          current = openSet[i];
          current_idx = i;
        }

      // if current = goal, then return reconstruct_path(came_from, goal)
      if(current->isGoal(goalNodePtr))
      {
        pds.m_iterations = openSet.size()+closedSet.size();
        CSolution<CNodeT> S(CSolution<CNodeT>::SUCCESS);
        current->getSolution(S.m_solution);
        return S;
      }

      // remove current from openset
      std::iter_swap(openSet.begin()+current_idx, openSet.end()-1);
      openSet.pop_back();

      // add current to closedset
      closedSet.push_back(current);

      // for each neighbor in neighbor_nodes(current)
      for(size_t i=0; i<current->m_childPtr.size(); i++)
      {
        // if neighbor in closedset, continue
        {
          bool found = false;
          for(size_t j=0; j<closedSet.size(); j++)
            if(closedSet[j] == current->m_childPtr[i]) { found = true; break; }
          if(found) continue;
        }

        //tentative_g_score := g_score[current] + dist_between(current,neighbor)
        Real tentative_g_score = g_score[current] + current->getDistToChild(i);

        // if neighbor not in openset or tentative_g_score < g_score[neighbor]
        {
          bool isChildInOpenSet = false;
          for(size_t j=0; j<openSet.size(); j++)
            if(current->m_childPtr[i] == openSet[j]) { isChildInOpenSet = true; break; }

          if( not isChildInOpenSet or tentative_g_score < g_score[current->m_childPtr[i]] )
          {
            current->m_childPtr[i]->compute();
            g_score[current->m_childPtr[i]] = tentative_g_score;
            f_score[current->m_childPtr[i]] = tentative_g_score + current->m_childPtr[i]->getHeuristicCostToGoal(goalNodePtr);
            if(not isChildInOpenSet) openSet.push_back(current->m_childPtr[i]);
          }
        }
      }
    }
    CSolution<CNodeT> F(CSolution<CNodeT>::FAILURE);
    return F;
  }

  inline bool
  refinePoses_AStarSearch
  (
    ShapeVector& shapes, // input: Shapes
    PoseVector& poses, // input: Poses
    IDVector& ID, // input: IDs
    PoseVector &refined_poses, // output: Refined poses
    ProblemDataStruct& pds
  )
  {
    refined_poses = poses;
    Matrix<Real,3,1> color(0.3f, 0.3f, 0.9f);

    AstarNode initNode(poses, 0.0f, NULL, &shapes, &ID);

    common::timestamp_t t0, t1;
    cerr << "A-star search started...\n";
    t0 = common::get_timestamp();
    CSolution<AstarNode> S = A_Star_Search(&initNode, &initNode, pds);
    t1 = common::get_timestamp();
    double dt = (double) (t1 - t0) / 1000000.0L;
    cerr << "A-star search finished in " << dt << " seconds\n";

		if( S.isSuccess() )
		{
		  cerr << "A-star search found a goal state in "<<S.m_solution.size()<<" steps\n";
		  refined_poses = S.m_solution[0]->m_poses;
      pds.m_steps = S.m_solution.size();
      pds.m_exeTime = dt;
      if(pds.m_glNodePtr != NULL)
      {
        for(int i=S.m_solution.size()-1; i>=0; i--)
        {
          pds.m_glNodePtr->ResetCMDbuffer();
          S.m_solution[i]->printNodeInfo(pds.m_glNodePtr);
          common::wait(0.1);
        }
      }
		}
		else
		{
			cerr << "A-star search terminated. Did not find goal state\n";
			return(false);
		}

    return(true);
  }

  }; // namespace promts
}; // namespace rasoul
#endif
