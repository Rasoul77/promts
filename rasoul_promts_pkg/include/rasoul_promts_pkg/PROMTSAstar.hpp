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
    AstarNode* m_parentNodePtr;                // Parent node pointer
    std::vector<AstarNode*> m_childsNodePtrs;  // Children node pointers

    PoseVector m_poses; // Node's poses

    Real m_costToReachThisNodeFromParent; // Cost to reach this node from parent node

    // Helper pointers to shapes and id's of the objects for computation
    ShapeVector* m_shapesPtr;
    IDVector*    m_IDPtr;

    // Constructors
    AstarNode() { m_isNodeAGoal = false; m_costToReachThisNodeFromParent = 0; }
    AstarNode
    (
      PoseVector&  poses,
      Real         costToReachThisNodeFromParent,
      AstarNode*   parentPtr,
      ShapeVector* shapesPtr,
      IDVector*    IDPtr
    )
    {
      m_isNodeAGoal = false;
      m_poses       = poses;
      m_costToReachThisNodeFromParent = costToReachThisNodeFromParent;
      m_parentNodePtr = parentPtr;
      m_shapesPtr     = shapesPtr;
      m_IDPtr         = IDPtr;
    }

    // Deconstructor
    ~AstarNode()
    {
      for(size_t i=0; i<m_childsNodePtrs.size(); i++)
        delete m_childsNodePtrs[i];
    }

    // Get distance of "this" node to idx-th child node
    Real getDistToChild( unsigned int idx )
    {
      if(idx<m_costFromThisToChild.size())
      {
        return(m_costFromThisToChild[idx]);
      }
      return(0);
    }

    // Get heuristic cost to reach a goal state from "this" node
    Real getHeuristicCostToGoal( AstarNode* goalNodePtr )
    {
      return(m_estimatedCostToGoal);
    }

    // Is "this" node the same as theNodePtr
    bool isSameNode( AstarNode* theNodePtr )
    {
      float sum = 0;
      size_t N = this->m_poses.size();
      for(size_t i=0; i<N; i++)
      {
        float dx = theNodePtr->m_poses[i].data()[TX]-this->m_poses[i].data()[TX];
        float dy = theNodePtr->m_poses[i].data()[TY]-this->m_poses[i].data()[TY];
        float dz = theNodePtr->m_poses[i].data()[TZ]-this->m_poses[i].data()[TZ];
        sum += dx*dx+dy*dy+dz*dz;
      }
      if(sum < 1e-4) return(true);
      return(false);
    }

    // Is "this" node a goal state?
    bool isGoal( AstarNode* goalNodePtr )
    {
      if(m_isNodeAGoal) return(true);
      return(false);
    }

    // Generate children nodes and compute the heuristic score
    void compute()
    {
      std::vector< geometry::AABBox<Real> > aabb(m_shapesPtr->size());
      m_estimatedCostToGoal = 0.0f;
      // Revert all the shapes to their original poses
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        m_shapesPtr->at(i).RevertPose();
        m_shapesPtr->at(i).TransformShape(m_poses[i]);
      }
      // Compute AABBs
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        geometry::bbComputePointsAABB(m_shapesPtr->at(i).V, aabb[i]);
        geometry::bbExtendAABB(aabb[i], (Real) 0.01);
      }
      // Compute DOPs
      PoseVector newPoses;
      for(size_t i=0; i<m_shapesPtr->size(); i++)
      {
        for(size_t j=i+1; j<m_shapesPtr->size(); j++)
        {
          // If two objects are fixed, skip!
          if(m_IDPtr->at(i) <= 0 && m_IDPtr->at(j) <= 0) continue;
          // If two objects AABBs collide, compute their possible DOP
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
                  m_childsNodePtrs.push_back(newNodePtr);
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
                  m_childsNodePtrs.push_back(newNodePtr);
                }
              }
            }
          }
        }
      }
      if(m_estimatedCostToGoal < 1e-4) m_isNodeAGoal = true;
    }

    // Return the solution
    void getSolution(std::vector<AstarNode*>& solution)
    {
      solution.clear();
      AstarNode* node = this;
      solution.push_back(node);
      while(node->m_parentNodePtr!=NULL)
      {
        node = node->m_parentNodePtr;
        solution.push_back(node);
      }
    }

    // Check whether new set of poses is already visited
    bool isVisited(PoseVector& newPoses)
    {
      AstarNode* theparent = m_parentNodePtr;
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
        if(sum < 1e-4) return(true);
        theparent = theparent->m_parentNodePtr;
      }
      return(false);
    }

    // Print some info about the node
    void printNodeInfo(COpenGLRosCom* glNodePtr)
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

  template<typename CNodeT, typename RealT>
  CSolution<CNodeT> A_Star_Search(CNodeT* startNodePtr, CNodeT* goalNodePtr, ProblemDataStruct& pds)
  {
    std::vector<CNodeT*> closedSet;     // The set of nodes already evaluated.
    std::vector<CNodeT*> openSet;       // The set of tentative nodes to be evaluated,
    openSet.push_back(startNodePtr);    // Initially containing the start node
    std::map<CNodeT*, RealT> g_score;   // Cost from start along best known path.
    g_score[startNodePtr] = 0;          // Initial node g_score is zero
    std::map<CNodeT*, RealT> f_score;   // Estimated total cost from start to goal through path y.

    startNodePtr->compute();            // Compute the heuristic cost to goal of start node, and generate successors

    f_score[startNodePtr] = g_score[startNodePtr] +
                            startNodePtr->getHeuristicCostToGoal(goalNodePtr);

    while(not openSet.empty())
    {
      // Give up if the number of expanded nodes are greater than some threshold
      if(openSet.size()+closedSet.size() > (unsigned) pds.m_maxNodes)
      {
        pds.m_iterations = openSet.size()+closedSet.size();
        CSolution<CNodeT> S(CSolution<CNodeT>::FAILURE);
        return S;
      }

      // Find the node in openSet having the lowest f_score[] value, label it as currentNodePtr
      size_t current_idx = 0;
      CNodeT* currentNodePtr = openSet[current_idx];
      RealT lowest_f_score = f_score[openSet[current_idx]];
      for(size_t i=1; i<openSet.size(); i++)
        if(f_score[openSet[i]] < lowest_f_score)
        {
          lowest_f_score = f_score[openSet[i]];
          currentNodePtr = openSet[i];
          current_idx = i;
        }

      // If currentNodePtr is a goal, then return solution
      if(currentNodePtr->isGoal(goalNodePtr))
      {
        pds.m_iterations = openSet.size()+closedSet.size();
        CSolution<CNodeT> S(CSolution<CNodeT>::SUCCESS);
        currentNodePtr->getSolution(S.m_solution);
        return S;
      }

      // Remove currentNodePtr from openset
      std::iter_swap(openSet.begin()+current_idx, openSet.end()-1);
      openSet.pop_back();

      // Add currentNodePtr to closedset
      closedSet.push_back(currentNodePtr);

      // For each child of currentNodePtr, do
      for(size_t i=0; i<currentNodePtr->m_childsNodePtrs.size(); i++)
      {
        // If child is in closedSet, continue
        {
          bool found = false;
          for(size_t j=0; j<closedSet.size(); j++)
            if(closedSet[j]->isSameNode(currentNodePtr->m_childsNodePtrs[i])) { found = true; break; }
          if(found) continue;
        }

        // Update tentative_g_score
        RealT tentative_g_score = g_score[currentNodePtr] + currentNodePtr->getDistToChild(i);

        {
          // if (child is not in openSet),
          bool isChildInOpenSet = false;
          for(size_t j=0; j<openSet.size(); j++)
            if(openSet[j]->isSameNode(currentNodePtr->m_childsNodePtrs[i])) { isChildInOpenSet = true; break; }
          // Add child to openSet
          if(not isChildInOpenSet) openSet.push_back(currentNodePtr->m_childsNodePtrs[i]);
          // else if (tentative_g_score >= g_score[current->m_childsNodePtrs[i]] ),
          else if( tentative_g_score >= g_score[currentNodePtr->m_childsNodePtrs[i]] )
            continue; // This is not a better path

          // Compute the successors and heuristic cost to goal of this child node
          currentNodePtr->m_childsNodePtrs[i]->compute();

          // Update g_score and f_score of this child node
          g_score[currentNodePtr->m_childsNodePtrs[i]] = tentative_g_score;
          f_score[currentNodePtr->m_childsNodePtrs[i]] = tentative_g_score + currentNodePtr->m_childsNodePtrs[i]->getHeuristicCostToGoal(goalNodePtr);
        }
      }
    }; // while()
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
    CSolution<AstarNode> S = A_Star_Search<AstarNode, Real>(&initNode, &initNode, pds);
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
