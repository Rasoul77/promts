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

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <limits>

#include "rasoul_promts_pkg/PROMTSAstar.hpp"
#include "rasoul_common_pkg/AnyOption.hpp"
#include "rasoul_common_pkg/Misc.hpp"
#include "rasoul_visualizer_pkg/OpenGLRosCom.hpp"
#include "rasoul_geometry_pkg/GeometryTransform.hpp"

#include "DemoHelper.hpp"

using namespace rasoul;

//+----------------------------------------------------------------------------+
//| main()                                                                     |
//-----------------------------------------------------------------------------+
int
main(int argc, char** argv)
{
  common::PathFileString pfs(argv[0]);
  //////////////////////////////////////////////////////////////////////////////
  // User command line parser
  AnyOption *opt = new AnyOption();
  opt->autoUsagePrint(true);

  opt->addUsage( "" );
  opt->addUsage( "Usage: " );
  char buff[256];
  sprintf(buff, "       Example: %s -r example1_noisy.cfg", pfs.getFileName().c_str());
  opt->addUsage( buff );
  opt->addUsage( " -h  --help                 Prints this help" );
  opt->addUsage( " -r  --readfile <filename>  Reads the polyhedra description file" );
  opt->addUsage( " -t  --gltopic <topic name> (opt) ROS Topic to send OpenGL commands " );
  opt->addUsage( "" );

  opt->setFlag(  "help", 'h' );
  opt->setOption(  "readfile", 'r' );

  opt->processCommandArgs( argc, argv );

  if( opt->getFlag( "help" ) || opt->getFlag( 'h' ) ) {opt->printUsage(); delete opt; return(1);}

  std::string gltopic("OpenGLRosComTopic");
  if( opt->getValue( 't' ) != NULL  || opt->getValue( "gltopic" ) != NULL  ) gltopic = opt->getValue( 't' );
	std::cerr << "[OpenGL communication topic is set to : \"" << gltopic << "\"]\n";

  std::string readfile;
	if( opt->getValue( 'r' ) != NULL  || opt->getValue( "readfile" ) != NULL  )
	{
    readfile = opt->getValue( 'r' );
    std::cerr << "[ World description is read from \"" << readfile << "\"]\n";
	}
	else{opt->printUsage(); delete opt; return(-1);}

  delete opt;
  //
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Initialize ROS and OpenGLRosCom node (visualization)
  std::cerr << "["<<pfs.getFileName()<<"]:" << "[Initializing ROS]"<< std::endl;
  ros::init(argc, argv, pfs.getFileName().c_str());
  if(ros::isInitialized())
    std::cerr << "["<<pfs.getFileName()<<"]:" << "[Initializing ROS]:[OK]\n"<< std::endl;
  else{
    std::cerr << "["<<pfs.getFileName()<<"]:" << "[Initializing ROS]:[FAILED]\n"<< std::endl;
    return(1);
  }
  COpenGLRosCom glNode;
  glNode.CreateNode(gltopic);
  //
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Reading the input file
  std::cerr << "Reading the input file..." << std::endl;
  ShapeVector shapes;
  PoseVector  poses;
  IDVector    ID;
  if(ReadPolyhedraConfigFile(readfile, shapes, poses, ID)==false)
  {
    std::cerr << "["<<pfs.getFileName()<<"]:" << "Error reading file \"" << readfile << "\"\n";
    return(-1);
  }
  std::cerr << "Read " << poses.size() << " poses of " << shapes.size() << " shapes with " << ID.size() << " IDs.\n";
  //
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Visualizing the configuration of objects
  std::cerr << "Visualizing the configuration of objects..." << std::endl;
  for(unsigned int i=0; i<shapes.size(); i++)
  {
    for(size_t j=0; j<shapes[i].F.size(); j++)
    {
      glNode.LineWidth(1.0f);
      glNode.AddColor3(0.3f, 0.6f, 0.5f);
      glNode.AddBegin("LINE_LOOP");
      for(size_t k=0; k<shapes[i].F[j].Idx.size(); k++)
      {
        int idx = shapes[i].F[j].Idx[k];
        Eigen::Matrix<Real,3,1> Vt = poses[i]*shapes[i].V[ idx ];
        glNode.AddVertex(Vt.x(), Vt.y(), Vt.z());
      }
      glNode.AddEnd();
    }
  }
  glNode.SendCMDbuffer();
  ros::spinOnce();
  common::PressEnterToContinue();
  //
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Running PROMTS with A* search algorithm
  std::cerr << "Running PROMTS with A* search algorithm..." << std::endl;
  promts::ProblemDataStruct pds("A* Search");
  pds.m_glNodePtr = &glNode;
  PoseVector refined_poses(shapes.size());
  promts::refinePoses_AStarSearch(shapes, poses, ID, refined_poses, pds);
  //
  //////////////////////////////////////////////////////////////////////////////

  return(0);
}
