#ifndef _OPENGL_ROS_COM_HPP_
#define _OPENGL_ROS_COM_HPP_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <queue>

#include <rasoul_visualizer_pkg/VizGL.h>
#include <rasoul_common_pkg/Misc.hpp>
#include <rasoul_common_pkg/TimeFunctions.hpp>

class COpenGLRosCom
{
  public:

    COpenGLRosCom() : nodeIsCreated_(false) {}

    ~COpenGLRosCom(){
      if(nodeIsCreated_) delete nh_ptr_;
    }

    void CreateNode(const std::string& topic_name)
    {
      if(nodeIsCreated_) return;
      nh_ptr_ = new ros::NodeHandle;
      pub_ = nh_ptr_->advertise<rasoul_visualizer_pkg::VizGL>(topic_name, 10);
      //common::wait(1.0);
      ros::Rate poll_rate(100);
      while(pub_.getNumSubscribers() == 0)
      {
        poll_rate.sleep();
      };
      nodeIsCreated_ = true;
    }

    void PushBuffer(void)
    {
      vizStack_.push(vizgl_);
    }

    void PopBuffer(void)
    {
      if(not vizStack_.empty())
      {
        vizgl_ = vizStack_.front();
        vizStack_.pop();
      }
    }

    void EnableGLTexture2D()
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glEnable_GL_TEXTURE_2D;
      vizgl_.cmd.push_back(glcmd);
    }

    void DisableGLTexture2D()
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glDisable_GL_TEXTURE_2D;
      vizgl_.cmd.push_back(glcmd);
    }

    void Texturize(unsigned int index)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glTexturize;
      glcmd.data.resize(1);
      glcmd.data[0] = float(index);
      vizgl_.cmd.push_back(glcmd);
    }

    void DrawAxis(float x=1.0f,float y=1.0f,float z=1.0f)
    {
      Eigen::Vector3f origin(Eigen::Vector3f::Zero());
      Eigen::Vector3f X(Eigen::Vector3f::UnitX());
      Eigen::Vector3f Y(Eigen::Vector3f::UnitY());
      Eigen::Vector3f Z(Eigen::Vector3f::UnitZ());

      AddColor3(x, 0.0, 0.0);
      DrawVector(origin, X);
      AddColor3(0.0, y, 0.0);
      DrawVector(origin, Y);
      AddColor3(0.0, 0.0, z);
      DrawVector(origin, Z);
    }

    void DrawVector(const Eigen::Vector3f& startPt,
                    const Eigen::Vector3f& endPt,
                    float base = 0.05f,
                    float height = 0.1f,
                    float slices = 10.0f,
                    float stacks = 10.0f)
    {
      AddLineSeg(startPt.x(), startPt.y(), startPt.z(),
                 endPt.x(), endPt.y(), endPt.z());
      Eigen::Vector3f F = endPt - startPt;
      Eigen::Vector3f rotAxis = Eigen::Vector3f::UnitZ().cross(F);
      double	sine = rotAxis.norm();
      double	cosine = Eigen::Vector3f::UnitZ().dot(F);
      bool	doRotation = true;
      if (fabs(sine) < 1e-5)
      {
        // rotAxis == zero vector. This means desired axis is either
        // along +z or -z. Hence:
        if (cosine > 0.0)
          doRotation = false; // cone axis already along +z
        else
        {
          // cone axis is along -z; just rotate by 180 degrees about
          // either x or y axis:
          rotAxis = Eigen::Vector3f::UnitX();
          // Just to eliminate roundoff error:
          sine = 0.0;
          cosine = -1.0;
        }
      }
      PushMatrix();
      Translate(endPt.x(), endPt.y(), endPt.z());
      if (doRotation)
      {
        double	angle = atan2(sine,cosine);
        Rotate(angle*180.0f/3.1415926535897f, rotAxis.x(), rotAxis.y(),rotAxis.z());
      }
      AddSolidCone(base,height,slices,stacks);
      PopMatrix();
    }

    void AddSolidCone(float base, float height, float slices, float stacks)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glutSolidCone;
      glcmd.data.resize(4);
      glcmd.data[0] = base;
      glcmd.data[1] = height;
      glcmd.data[2] = slices;
      glcmd.data[3] = stacks;
      vizgl_.cmd.push_back(glcmd);
    }

    void Translate(float x, float y, float z)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glTranslatef;
      glcmd.data.resize(3);
      glcmd.data[0] = x;
      glcmd.data[1] = y;
      glcmd.data[2] = z;
      vizgl_.cmd.push_back(glcmd);
    }

    void Rotate(float angle, float x, float y, float z)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glRotatef;
      glcmd.data.resize(4);
      glcmd.data[0] = angle;
      glcmd.data[1] = x;
      glcmd.data[2] = y;
      glcmd.data[3] = z;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddTextAt(const std::string& text,
                   float textWidth,
                   const Eigen::Vector3f& Tr,
                   const Eigen::Vector3f& R,
                   float angle,
                   const Eigen::Vector3f& Scale,
                   const Eigen::Vector3f& Color)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(14);
      glcmd.name = vizgl_.writeText;
      glcmd.data[0] = textWidth;
      glcmd.data[1] = Tr.x();
      glcmd.data[2] = Tr.y();
      glcmd.data[3] = Tr.z();
      glcmd.data[4] = R.x();
      glcmd.data[5] = R.y();
      glcmd.data[6] = R.z();
      glcmd.data[7] = angle;
      glcmd.data[8] = Scale.x();
      glcmd.data[9] = Scale.y();
      glcmd.data[10] = Scale.z();
      glcmd.data[11] = Color.x();
      glcmd.data[12] = Color.y();
      glcmd.data[13] = Color.z();
      glcmd.text = text;
      vizgl_.cmd.push_back(glcmd);
    }

    void ResetCMDbuffer(void)
    {
      while(not vizStack_.empty())
        vizStack_.pop();
      vizgl_.cmd.clear();
    }

    void SendCMDbuffer(void)
    {
      pub_.publish(vizgl_);
      for(size_t i=0; i<1000; i++) ros::spinOnce();
    }

    void AddCMD(const rasoul_visualizer_pkg::GLcmd& glcmd)
    {
      vizgl_.cmd.push_back(glcmd);
    }

    void AddBegin(const std::string& id)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      if(id.compare("POINTS") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_POINTS;
      }
      else if(id.compare("LINES") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_LINES;
      }
      else if(id.compare("LINE_STRIP") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_LINE_STRIP;
      }
      else if(id.compare("LINE_LOOP") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_LINE_LOOP;
      }
      else if(id.compare("TRIANGLES") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_TRIANGLES;
      }
      else if(id.compare("TRIANGLE_STRIP") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_TRIANGLE_STRIP;
      }
      else if(id.compare("TRIANGLE_FAN") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_TRIANGLE_FAN;
      }
      else if(id.compare("QUADS") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_QUADS;
      }
      else if(id.compare("QUAD_STRIP") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_QUAD_STRIP;
      }
      else if(id.compare("POLYGON") == 0)
      {
        glcmd.name = vizgl_.glBegin_GL_POLYGON;
      }
      vizgl_.cmd.push_back(glcmd);
    }

    void LineWidth(float thickness)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glLineWidth;
      glcmd.data.push_back(thickness);
      vizgl_.cmd.push_back(glcmd);
    }

    void PointSize(float size)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glPointSize;
      glcmd.data.push_back(size);
      vizgl_.cmd.push_back(glcmd);
    }

    void AddEnd()
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glEnd;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddNormal3f(float x, float y, float z)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(3);
      glcmd.name = vizgl_.glNormal3f;
      glcmd.data[0] = x; glcmd.data[1] = y;glcmd.data[2] = z;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddTexCoord2f(float x, float y)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(2);
      glcmd.name = vizgl_.glTexCoord2f;
      glcmd.data[0] = x; glcmd.data[1] = y;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddVertex(float x, float y, float z)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(3);
      glcmd.name = vizgl_.glVertex3f;
      glcmd.data[0] = x; glcmd.data[1] = y; glcmd.data[2] = z;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddColor3(float R, float G, float B)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(3);
      glcmd.name = vizgl_.glColor3f;
      glcmd.data[0] = R; glcmd.data[1] = G; glcmd.data[2] = B;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddColor4(float R, float G, float B, float alpha)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(4);
      glcmd.name = vizgl_.glColor4f;
      glcmd.data[0] = R; glcmd.data[1] = G; glcmd.data[2] = B; glcmd.data[3] = alpha;
      vizgl_.cmd.push_back(glcmd);
    }

    void PushMatrix()
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glPushMatrix;
      vizgl_.cmd.push_back(glcmd);
    }

    void PopMatrix()
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glPopMatrix;
      vizgl_.cmd.push_back(glcmd);
    }

    void MultMatrix(float* m)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.data.resize(16);
      for(size_t i=0; i<16; i++) glcmd.data[i] = m[i];
      glcmd.name = vizgl_.glMultMatrixf;
      vizgl_.cmd.push_back(glcmd);
    }

    void AddPoint(float x, float y, float z)
    {
      AddBegin("POINTS");
      AddVertex(x,y,z);
      AddEnd();
    }

    void AddLineSeg(float x0, float y0, float z0,
                    float x1, float y1, float z1)
    {
      AddBegin("LINES");
      AddVertex(x0,y0,z0);
      AddVertex(x1,y1,z1);
      AddEnd();
    }

    void LoadIdentity(void)
    {
      rasoul_visualizer_pkg::GLcmd glcmd;
      glcmd.name = vizgl_.glLoadIdentity;
      vizgl_.cmd.push_back(glcmd);
    }

  private:
    ros::NodeHandle*  nh_ptr_;
    ros::Publisher    pub_;
    rasoul_visualizer_pkg::VizGL vizgl_;
    //std::stack<rasoul_visualizer_pkg::VizGL> vizStack_;
    std::queue<rasoul_visualizer_pkg::VizGL> vizStack_;
    bool nodeIsCreated_;
};

#endif
