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

#include "DemoHelper.hpp"

bool
ReadPolyhedraConfigFile
(
  const std::string&   filename,
  ShapeVector& shapes,
  PoseVector& poses,
  IDVector& ID
)
{
  std::ifstream readfile (filename.c_str());

  std::vector< Eigen::Matrix<Real,3,1> > Vi;
  PolyhedraShape shape;
  std::string object_class_name;
  Real T[3]={0}, Q[4]={0};

  if (readfile.is_open())
  {

    std::string line;
    bool readline = true;
    while ( readfile.good() )
    {
      if(readline) std::getline (readfile, line);
      if(line[0] == '#' && line[1] == '#')
        continue;
      else if(line.compare("# Object Class Name") == 0)
      {
        std::getline (readfile, object_class_name);
      }
      else if(line.compare("# Object ID") == 0)
      {
        std::getline (readfile, line);
        ID.push_back(rasoul::common::StringToNumber<int> (line));
      }
      else if(line.compare("# Object Center Of Mass Position") == 0)
      {
        std::getline (readfile, line); T[0] = rasoul::common::StringToNumber<Real> (line);
        std::getline (readfile, line); T[1] = rasoul::common::StringToNumber<Real> (line);
        std::getline (readfile, line); T[2] = rasoul::common::StringToNumber<Real> (line);
      }
      else if(line.compare("# Object Orientation") == 0)
      {
        std::getline (readfile, line); Q[0] = rasoul::common::StringToNumber<Real> (line);
        std::getline (readfile, line); Q[1] = rasoul::common::StringToNumber<Real> (line);
        std::getline (readfile, line); Q[2] = rasoul::common::StringToNumber<Real> (line);
        std::getline (readfile, line); Q[3] = rasoul::common::StringToNumber<Real> (line);

        Eigen::Quaternion<Real> q(Q[3],Q[0],Q[1],Q[2]);
        Eigen::Translation<Real,3> t(T[0], T[1], T[2]);
        Eigen::Affine3f pose = t * q;
        poses.push_back(pose);
      }
      else if(line.compare("# Object Inverse Mass") == 0)
      {
        std::getline (readfile, line);
      }
      else if(line.compare("# Object Shape") == 0)
      {
        if(object_class_name.compare("Convex") == 0)
        {
          std::getline (readfile, line); int n = rasoul::common::StringToNumber<int> (line);
          Vi.clear()          ;
          for(int i=0; i<n; i++)
          {
            std::getline (readfile, line); Real X = rasoul::common::StringToNumber<Real> (line);
            std::getline (readfile, line); Real Y = rasoul::common::StringToNumber<Real> (line);
            std::getline (readfile, line); Real Z = rasoul::common::StringToNumber<Real> (line);
            Vi.push_back(Eigen::Matrix<Real,3,1>(X,Y,Z));
          }
          std::getline (readfile, line); int m = rasoul::common::StringToNumber<int> (line);
          for(int i=0; i<m; i++)
          {
            std::getline (readfile, line); int p = rasoul::common::StringToNumber<int> (line);
            for(int j=0; j<p; j++)
            {
              std::getline (readfile, line);
            }
          }
        }
        else{
          std::cerr<<"This feature has not been implemented yet!\n";
        }
      }
      else if(line.compare("# Object Texture ID") == 0)
      {
        std::getline (readfile, line);
        readline = true;
      }
      else if(line.compare("# Object Color") == 0)
      {
        std::getline (readfile, line);
        std::getline (readfile, line);
        std::getline (readfile, line);
      }
      else if(line.compare("# Object Alpha") == 0)
      {
        std::getline (readfile, line);
        #ifdef PROMTS_USE_BULLET
          shape.Compute(Vi,rasoul::geometry::CGeometryPolyhedra<Real>::METHOD_BULLET);
        #else
          shape.Compute(Vi,rasoul::geometry::CGeometryPolyhedra<Real>::METHOD_BRUTE_FORCE);
        #endif
        shapes.push_back(shape);
      }
      else if(line.compare("# Captured <Points-ObjectID>") == 0)
      {
        readfile.close();
        return(true);
      }
    }

    readfile.close();
    return(true);
  }
  return(false);
}
