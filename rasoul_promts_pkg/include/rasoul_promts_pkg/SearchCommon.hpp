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

#ifndef _SEARCH_COMMON_HPP_
#define _SEARCH_COMMON_HPP_
#include <limits>
#include <string>
#include "rasoul_visualizer_pkg/OpenGLRosCom.hpp"
#include "rasoul_geometry_pkg/GeometryPolyhedra.hpp"

namespace rasoul
{
  namespace promts
  {

typedef float Real;
typedef bool Flag;
typedef int Int;
typedef unsigned int Uint;
typedef Eigen::Transform<Real,3,Eigen::Affine> eTransform;
typedef std::vector< eTransform, Eigen::aligned_allocator<eTransform> > PoseVector;
typedef rasoul::geometry::CGeometryPolyhedra<Real> PolyhedraShape;
typedef std::vector< PolyhedraShape > ShapeVector;
typedef std::vector<int> IDVector;
typedef std::pair<int, int> IDPairs;

const unsigned int TX = 12, TY = 13, TZ = 14;

/** \brief Problem data structure.
  *
  * \ingroup planner
  */
struct ProblemDataStruct
{
  COpenGLRosCom* m_glNodePtr;
  Flag           m_visualize_enter;
  Real           m_exeTime;
  Int            m_steps;
  Int            m_iterations;
  Int            m_maxNodes;
  std::string    m_methodName;

  ProblemDataStruct(std::string methodName){
    m_glNodePtr  = NULL;
    m_visualize_enter = false;
    m_exeTime    = std::numeric_limits<Real>::infinity();
    m_steps      = -1;
    m_iterations = -1;
    m_maxNodes   = std::numeric_limits<Int>::max();
    m_methodName = methodName;
  }
};

/** \brief Solution class.
  *
  * \ingroup planner
  */
template<typename SearchNodeT>
class CSolution{
public:
  enum ENUM_RESULTS{CUTOFF=-1,FAILURE=0,SUCCESS=1,UNKNOWN=2};
  CSolution() { m_state = UNKNOWN; m_cost = std::numeric_limits<Real>::infinity();}
  CSolution(ENUM_RESULTS state) {m_state = state; m_cost = std::numeric_limits<Real>::infinity();}
  bool isCutoff(){if(m_state == CUTOFF) return(true); return(false);}
  bool isFailure(){if(m_state == FAILURE) return(true); return(false);}
  bool isSuccess(){if(m_state == SUCCESS) return(true); return(false);}

  std::vector<SearchNodeT*> m_solution;
  Real m_cost;
private:
  ENUM_RESULTS m_state;
};


  };
};
#endif
