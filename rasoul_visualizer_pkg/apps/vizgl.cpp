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
#include <X11/Xlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <rasoul_visualizer_pkg/VizGL.h>
#include <rasoul_common_pkg/AnyOption.hpp>

#include <GL/glut.h>
#include "imageloader.h"
#include "text3d.h"

#define MOUSE_WHEEL_FORWARD 3
#define MOUSE_WHEEL_BACKWARD 4

#define rotXY 0
#define rotYZ 1
#define rotZX 2

unsigned int g_frame_counter;
bool g_capture;

using namespace rasoul_visualizer_pkg;

struct Texture
{
  GLuint ID;
  string filename;
};

struct Vector3
{
  float x;
  float y;
  float z;
};

struct Global {

  bool bFullScreen;
  bool bKeyW;
  bool bKeyS;
  bool bRotate;
  bool bPan;
  bool bUpdated;

  int WinW;
  int WinH;
  unsigned int WinInitPx;
  unsigned int WinInitPy;
  int InitMousePoseX;
  int InitMousePoseY;
  int EndMousePoseX;
  int EndMousePoseY;
  int mouseState;
  int mouseButton;
  int callbacktimer_mSec;
  int WindowID;
  unsigned int rotMode;

  float Zoom;
  float AngleX;
  float AngleY;
  float AngleZ;
  float panX;
  float panY;

  vector<Texture> textures;

  VizGL vizgl;

} global;

bool readTextureFile(const std::string& filename);
GLuint loadTexture(Image* image);
void CallBackKeyboardFunc(unsigned char key, int x, int y);
void initRendering();
void registerCallBacks();
void handleResize(int w, int h);
void CallBackDisplayFunc();
void CallBackMouseFunc(int button, int state, int x, int y);
void CallBackMotionFunc(int x, int y);
void CallBackIdleFunc();
void CallBackReshapeFunc(int w, int h);
void CallBackTimerFunc(int value);
void rosCallback(const VizGL::ConstPtr& msg);
void drawGLcmd(const GLcmd& glcmd);
int  getRootWindowSize(int *w, int *h);
void drawAxis();
void writeTextAt(const GLcmd& glcmd);
void writeTextAt2(const GLcmd& glcmd);
void writeTextAt3(const GLcmd& glcmd);
void CaptureViewPort();

int main(int argc, char **argv)
{
  g_frame_counter = 0;
  g_capture = true;
  // ==========================================================================
  // command line parser
  // ==========================================================================
  AnyOption *opt = new AnyOption();

  opt->autoUsagePrint(true);

  opt->addUsage( "" );
  opt->addUsage( "Usage: " );
  opt->addUsage( "" );
  opt->addUsage( " -h  --help                   Prints this help " );
  opt->addUsage( " -x  --texturefile <filename> A text file containing the path to texture images " );
  opt->addUsage( " -c  --charsetfile <filename> Charset file name" );
  opt->addUsage( " -t  --gltopic <topic name>   (opt) ROS Topic to receive OpenGL commands " );
  opt->addUsage( "" );

  opt->setFlag(  "help", 'h' );
  opt->setOption(  "gltopic",    't' );
  opt->setOption(  "texturefile", 'x' );
  opt->setOption(  "charsetfile", 'c' );

  opt->processCommandArgs( argc, argv );

  std::cout << "argc = " << argc << std::endl;

  if( opt->getFlag( "help" ) || opt->getFlag( 'h' ) ) {std::cout << "(1)\n"; opt->printUsage(); delete opt; return(1);}

  std::string gltopic("OpenGLRosComTopic");
  if( opt->getValue( 't' ) != NULL  || opt->getValue( "gltopic" ) != NULL  ) gltopic = opt->getValue( 't' );
  std::cout << "[OpenGL communication topic is set to : \"" << gltopic << "\"]\n";

  std::string textureFile;
  if( opt->getValue( 'x' ) != NULL  || opt->getValue( "texturefile" ) != NULL  ) textureFile = opt->getValue( 'x' );
  if(textureFile.empty()) {std::cout << "[!!! ERROR !!!]:[texturefile not specified]:\n" << std::endl; opt->printUsage(); delete opt; return(1);}
  else
    std::cout << "[Texture images will be loaded from  \"" << textureFile << "\"]\n";

  std::string charsetFile;
  if( opt->getValue( 'c' ) != NULL  || opt->getValue( "charsetfile" ) != NULL  ) charsetFile = opt->getValue( 'c' );
  if(charsetFile.empty()) {std::cout << "[!!! ERROR !!!]:[charsetFile not specified]:\n" << std::endl; opt->printUsage(); delete opt; return(1);}
  else
    std::cout << "[charset file will be loaded from  \"" << charsetFile << "\"]\n";

  delete opt;

  // ==========================================================================
  // ROS initialization
  // ==========================================================================
  ros::init(argc, argv, "vizgl_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(gltopic, 10, rosCallback);


  // ==========================================================================
  // get screen properties from X11
  // ==========================================================================
  int ScWidth, ScHeight;
  if(!getRootWindowSize(&ScWidth, &ScHeight))
    std::cerr << "[Screen Resolution]: " << ScWidth << " x " << ScHeight << std::endl;
  else{
    ScWidth = 1024;
    ScHeight = 768;
    std::cerr << "[Screen Resolution]: Error! assumed to be " << ScWidth << " x " << ScHeight << std::endl;
  }

  // ==========================================================================
  // initialization of global variables
  // ==========================================================================
  global.bFullScreen = false;
  global.bKeyS = false;
  global.bKeyW = false;
  global.bRotate = false;
  global.bPan = false;
  global.bUpdated = false;
  global.WinW = 2*ScWidth/3;
  global.WinH = 2*ScHeight/3;
  global.WinInitPx = (ScWidth - global.WinW)/2;
  global.WinInitPy = (ScHeight - global.WinH)/2;
  global.InitMousePoseX = 0;
  global.InitMousePoseY = 0;
  global.EndMousePoseX = 0;
  global.EndMousePoseY = 0;
  global.rotMode = rotYZ;
  global.Zoom = -3.0f;
  global.AngleX = 0.0f;
  global.AngleY = 180.0f;
  global.AngleZ = 0.0f;


  // ==========================================================================
  // Reading texture file
  // ==========================================================================
  if(not readTextureFile(textureFile))
  {
    std::cerr << "Error in texture file!\n";
    return(1);
  }

  // ==========================================================================
  // OpenGL configuration
  // ==========================================================================
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(global.WinW, global.WinH);
  glutInitWindowPosition(global.WinInitPx, global.WinInitPy);
  global.WindowID = glutCreateWindow("VizGL");

  initRendering();

  if(not t3dInit(charsetFile))
  {
    std::cerr << "Error in charset file!\n";
    return(1);
  }

  registerCallBacks();
  glutTimerFunc(25, CallBackTimerFunc, 0);

  ros::spinOnce();
	glutMainLoop();

  return 0;
}

void rosCallback(const VizGL::ConstPtr& msg)
{
  global.vizgl = *msg;
  //std::cerr << "global.vizgl.cmd.size() = " << global.vizgl.cmd.size() << std::endl;
  g_capture = true;
}

void CallBackTimerFunc(int value)
{
  glutPostRedisplay();
  for(int i=0; i<100; i++)ros::spinOnce();
	glutTimerFunc(25, CallBackTimerFunc, 0);
}

void CallBackDisplayFunc(void)
{
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  if(global.bRotate)
  {
    float rotateX = 0, rotateY = 0, rotateZ = 0;
    if(global.rotMode == rotXY)
    {
      rotateX = global.EndMousePoseX - global.InitMousePoseX;
      rotateY = global.EndMousePoseY - global.InitMousePoseY;
    }
    else if(global.rotMode == rotYZ)
    {
      rotateZ = global.EndMousePoseX - global.InitMousePoseX;
      rotateY = global.EndMousePoseY - global.InitMousePoseY;
    }
    else if(global.rotMode == rotZX)
    {
      rotateX = global.EndMousePoseX - global.InitMousePoseX;
      rotateZ = global.EndMousePoseY - global.InitMousePoseY;
    }
    global.InitMousePoseX = global.EndMousePoseX;
    global.InitMousePoseY = global.EndMousePoseY;
    global.AngleY += rotateY;
    if (global.AngleY > 360.0f) global.AngleY -= 360.0f;
    if (global.AngleY < -360.0f) global.AngleY += 360.0f;
    global.AngleX += rotateX;
    if (global.AngleX > 360.0f) global.AngleX -= 360.0f;
    if (global.AngleX < -360.0f) global.AngleX += 360.0f;
    global.AngleZ += rotateZ;
    if (global.AngleZ > 360.0f) global.AngleZ -= 360.0f;
    if (global.AngleZ < -360.0f) global.AngleZ += 360.0f;
  }

  if(global.bPan)
  {
    global.panX += (global.EndMousePoseX - global.InitMousePoseX)/100.0f;
    global.panY += (global.EndMousePoseY - global.InitMousePoseY)/100.0f;
    global.InitMousePoseX = global.EndMousePoseX;
    global.InitMousePoseY = global.EndMousePoseY;
  }

  glTranslatef(global.panX,-global.panY,global.Zoom);
  if(global.rotMode == rotXY)
  {
    glRotatef(global.AngleX, 0.0f, 1.0f, 0.0f);
    glRotatef(global.AngleY, 1.0f, 0.0f, 0.0f);
  }
  else if(global.rotMode == rotYZ)
  {
    glRotatef(global.AngleY, 1.0f, 0.0f, 0.0f);
    glRotatef(global.AngleZ, 0.0f, 0.0f, 1.0f);
  }
  else if(global.rotMode == rotZX)
  {
    glRotatef(global.AngleX, 0.0f, 1.0f, 0.0f);
    glRotatef(global.AngleZ, 0.0f, 0.0f, 1.0f);
  }

	GLfloat ambientColor[] = {1.5f, 1.5f, 1.5f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

  for(size_t i=0; i<global.vizgl.cmd.size(); i++)
  {
    drawGLcmd(global.vizgl.cmd[i]);
  }

  glutSwapBuffers();

  if(g_capture) {CaptureViewPort(); g_capture = false;}
}

void drawGLcmd(const GLcmd& glcmd)
{
  VizGL vizGL;
  if(glcmd.name == vizGL.glBegin_GL_POINTS)
    glBegin(GL_POINTS);
  else if(glcmd.name == vizGL.glBegin_GL_LINES)
    glBegin(GL_LINES);
  else if(glcmd.name == vizGL.glBegin_GL_LINE_STRIP)
    glBegin(GL_LINE_STRIP);
  else if(glcmd.name == vizGL.glBegin_GL_LINE_LOOP)
    glBegin(GL_LINE_LOOP);
  else if(glcmd.name == vizGL.glBegin_GL_TRIANGLES)
    glBegin(GL_TRIANGLES);
  else if(glcmd.name == vizGL.glBegin_GL_TRIANGLE_STRIP)
    glBegin(GL_TRIANGLE_STRIP);
  else if(glcmd.name == vizGL.glBegin_GL_TRIANGLE_FAN)
    glBegin(GL_TRIANGLE_FAN);
  else if(glcmd.name == vizGL.glBegin_GL_QUADS)
    glBegin(GL_QUADS);
  else if(glcmd.name == vizGL.glBegin_GL_QUAD_STRIP)
    glBegin(GL_QUAD_STRIP);
  else if(glcmd.name == vizGL.glBegin_GL_POLYGON)
    glBegin(GL_POLYGON);
  else if(glcmd.name == vizGL.glEnd)
    glEnd();
  else if(glcmd.name == vizGL.glVertex3f)
    glVertex3f(glcmd.data[0], glcmd.data[1], glcmd.data[2]);
  else if(glcmd.name == vizGL.glColor3f)
    glColor3f(glcmd.data[0], glcmd.data[1], glcmd.data[2]);
  else if(glcmd.name == vizGL.glColor4f)
    glColor4f(glcmd.data[0], glcmd.data[1], glcmd.data[2], glcmd.data[3]);
  else if(glcmd.name == vizGL.glPushMatrix)
    glPushMatrix();
  else if(glcmd.name == vizGL.glPopMatrix)
    glPopMatrix();
  else if(glcmd.name == vizGL.glLineWidth)
    glLineWidth(glcmd.data[0]);
  else if(glcmd.name == vizGL.glMultMatrixf)
  {
    float m[16];
    for(size_t i=0; i<16; i++) m[i] = glcmd.data[i];
    glMultMatrixf(m);
  }
  else if(glcmd.name == vizGL.glPointSize)
    glPointSize( glcmd.data[0] );
  else if(glcmd.name == vizGL.writeText)
    writeTextAt(glcmd);
  else if(glcmd.name == vizGL.writeText2)
    writeTextAt2(glcmd);
  else if(glcmd.name == vizGL.writeText3)
    writeTextAt3(glcmd);
  else if(glcmd.name == vizGL.glLoadIdentity)
    glLoadIdentity();
  else if(glcmd.name == vizGL.glutSolidCone)
    glutSolidCone(glcmd.data[0],glcmd.data[1],(int)glcmd.data[2],(int)glcmd.data[3]);
  else if(glcmd.name == vizGL.glTranslatef)
    glTranslatef(glcmd.data[0], glcmd.data[1], glcmd.data[2]);
  else if(glcmd.name == vizGL.glRotatef)
    glRotatef(glcmd.data[0],glcmd.data[1],glcmd.data[2],glcmd.data[3]);
  else if(glcmd.name == vizGL.glTexturize)
  {
    glBindTexture(GL_TEXTURE_2D, global.textures[(unsigned int)glcmd.data[0]].ID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  }
  else if(glcmd.name == vizGL.glTexCoord2f)
    glTexCoord2f(glcmd.data[0], glcmd.data[1]);
  else if(glcmd.name == vizGL.glDisable_GL_TEXTURE_2D)
    glDisable(GL_TEXTURE_2D);
  else if(glcmd.name == vizGL.glEnable_GL_TEXTURE_2D)
    glEnable(GL_TEXTURE_2D);
  else if(glcmd.name == vizGL.drawSolidCylinder)
  {
    float radius = glcmd.data[0];
    float halfHeight = glcmd.data[1] / 2.0f;
    uint16_t N = 36;
    float stepAlpha = ( 2.0f / (float) N ) * M_PI;
    // TOP CIRCLE
    std::vector<Vector3> topPoints(N);
    glBegin(GL_POLYGON);
    for(size_t i=0; i<N; i++)
    {
      float theta = (float) i * stepAlpha;
      topPoints[i].x =  radius*cos(theta);
      topPoints[i].y =  halfHeight;
      topPoints[i].z = -radius*sin(theta);
      glVertex3f(topPoints[i].x, topPoints[i].y, topPoints[i].z);
    }
    glEnd();
    // BOTTOM CIRCLE
    std::vector<Vector3> bottomPoints(N);
    glBegin(GL_POLYGON);
    for(size_t i=0; i<N; i++)
    {
      bottomPoints[i].x =  topPoints[i].x;
      bottomPoints[i].y = -topPoints[i].y;
      bottomPoints[i].z =  topPoints[i].z;
      glVertex3f(bottomPoints[i].x, bottomPoints[i].y, bottomPoints[i].z);
    }
    glEnd();
    glBegin(GL_QUAD_STRIP);
    glVertex3f(topPoints[0].x, topPoints[0].y, topPoints[0].z);
    glVertex3f(bottomPoints[0].x, bottomPoints[0].y, bottomPoints[0].z);
    for(size_t i=1; i<N; i++)
    {
      glVertex3f(topPoints[i].x, topPoints[i].y, topPoints[i].z);
      glVertex3f(bottomPoints[i].x, bottomPoints[i].y, bottomPoints[i].z);
    }
    glEnd();
  }
  else if(glcmd.name == vizGL.drawWiredCylinder)
  {
    float radius = glcmd.data[0];
    float halfHeight = glcmd.data[1] / 2.0f;
    uint16_t N = 36;
    float stepAlpha = ( 2.0f / (float) N ) * M_PI;
    // TOP CIRCLE
    std::vector<Vector3> topPoints(N);
    glBegin(GL_LINE_LOOP);
    for(size_t i=0; i<N; i++)
    {
      float theta = (float) i * stepAlpha;
      topPoints[i].x =  radius*cos(theta);
      topPoints[i].y =  halfHeight;
      topPoints[i].z = -radius*sin(theta);
      glVertex3f(topPoints[i].x, topPoints[i].y, topPoints[i].z);
    }
    glEnd();
    // BOTTOM CIRCLE
    std::vector<Vector3> bottomPoints(N);
    glBegin(GL_LINE_LOOP);
    for(size_t i=0; i<N; i++)
    {
      bottomPoints[i].x =  topPoints[i].x;
      bottomPoints[i].y = -topPoints[i].y;
      bottomPoints[i].z =  topPoints[i].z;
      glVertex3f(bottomPoints[i].x, bottomPoints[i].y, bottomPoints[i].z);
    }
    glEnd();
    glBegin(GL_LINES);
    for(size_t i=0; i<N; i++)
    {
      glVertex3f(topPoints[i].x, topPoints[i].y, topPoints[i].z);
      glVertex3f(bottomPoints[i].x, bottomPoints[i].y, bottomPoints[i].z);
    }
    glEnd();
  }
}

void CallBackMouseFunc(int button, int state, int x, int y)
{
  global.mouseButton = button;
  global.mouseState  = state;
  switch(button)
  {
    case GLUT_LEFT_BUTTON:
      switch(state)
      {
        case GLUT_DOWN:
          global.InitMousePoseX = x;
          global.InitMousePoseY = y;

        break;
        case GLUT_UP:
          global.bRotate = false;
        break;
      }
    break;
    case GLUT_MIDDLE_BUTTON:
      if(global.rotMode == rotXY) global.rotMode = rotYZ;
      else if(global.rotMode == rotYZ) global.rotMode = rotZX;
      else global.rotMode = rotXY;
    break;
    case GLUT_RIGHT_BUTTON:
      switch(state)
      {
        case GLUT_DOWN:
          global.InitMousePoseX = x;
          global.InitMousePoseY = y;
        break;
        case GLUT_UP:
          global.bPan = false;
        break;
      }
    break;
    case MOUSE_WHEEL_FORWARD:
      global.Zoom += 0.10;
    break;
    case MOUSE_WHEEL_BACKWARD:
      global.Zoom -= 0.10;
    break;
    default:
    break;
  }
}

void CallBackMotionFunc(int x, int y)
{
  global.EndMousePoseX = x;
  global.EndMousePoseY = y;
  if( global.mouseButton == GLUT_LEFT_BUTTON ||
      global.mouseButton == GLUT_MIDDLE_BUTTON ) global.bRotate = true;
  if( global.mouseButton == GLUT_RIGHT_BUTTON ) global.bPan = true;
}

void CallBackKeyboardFunc(unsigned char key, int x, int y)
{
  switch(key)
  {
    case 27://ESC
      ROS_INFO("Exit!\n");
      glutDestroyWindow(global.WindowID);
      exit(0);
    break;
    case 'f':
      if(global.bFullScreen)
      {
        global.bFullScreen = false;
        glutReshapeWindow(global.WinW, global.WinH);
        glutPositionWindow(global.WinInitPx, global.WinInitPy);
      }
      else{
        global.bFullScreen = true;
        glutFullScreen();
      }
    break;
    case 'w':
      if(global.bKeyW)
        global.bKeyW = false;
      else
        global.bKeyW = true;
    break;
    case 's':
      if(global.bKeyS)
        global.bKeyS = false;
      else
        global.bKeyS = true;
      break;
    break;
  }
}

void initRendering()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_MULTISAMPLE);

  for(unsigned int i=0; i<global.textures.size(); i++)
  {
    Image* image = loadBMP(global.textures[i].filename.c_str());
    global.textures[i].ID = loadTexture(image);
  }

}

void registerCallBacks()
{
  glutDisplayFunc(CallBackDisplayFunc);
  glutKeyboardFunc(CallBackKeyboardFunc);
  glutReshapeFunc(CallBackReshapeFunc);
  glutMouseFunc(CallBackMouseFunc);
  glutMotionFunc(CallBackMotionFunc);
}

void handleResize(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (double)w / (double)h, 0.001, 200.0);
}

void writeTextAt(const GLcmd& glcmd)
{

  enum {TextWidth, Tx, Ty, Tz, Rx, Ry, Rz, Angle,
        ScaleX, ScaleY, ScaleZ, ColorX, ColorY, ColorZ};

  glPushMatrix();
  glTranslatef(glcmd.data[Tx], glcmd.data[Ty], glcmd.data[Tz]);
  glRotatef(glcmd.data[Angle],glcmd.data[Rx],glcmd.data[Ry],glcmd.data[Rz]);
  glScalef(glcmd.data[ScaleX], glcmd.data[ScaleY], glcmd.data[ScaleY]);
  glColor3f(glcmd.data[ColorX], glcmd.data[ColorY], glcmd.data[ColorZ]);
  t3dDraw3D(glcmd.text, 0, 0, 0.01f);
  glPopMatrix();
}

void writeTextAt2(const GLcmd& glcmd)
{

  enum {TextWidth, Tx, Ty, Tz, roll, pitch, yaw,
        ScaleX, ScaleY, ScaleZ, ColorX, ColorY, ColorZ};

  glPushMatrix();
  glTranslatef(glcmd.data[Tx], glcmd.data[Ty], glcmd.data[Tz]);
  glRotatef(glcmd.data[roll],1.0f, 0.0f, 0.0f);
  glRotatef(glcmd.data[pitch],0.0f, 1.0f, 0.0f);
  glRotatef(glcmd.data[yaw],0.0f, 0.0f, 1.0f);
  glScalef(glcmd.data[ScaleX], glcmd.data[ScaleY], glcmd.data[ScaleY]);
  glColor3f(glcmd.data[ColorX], glcmd.data[ColorY], glcmd.data[ColorZ]);
  t3dDraw3D(glcmd.text, 0, 0, 0.01f);
  glPopMatrix();
}

void writeTextAt3(const GLcmd& glcmd)
{
  glPushMatrix();
  float m[16];
  for(unsigned int j=0; j<16; j++) m[j] = glcmd.data[j];

  unsigned int i = 16;
  float x1 = glcmd.data[i++];
  float y1 = glcmd.data[i++];
  float z1 = glcmd.data[i++];
  float x2 = glcmd.data[i++];
  float y2 = glcmd.data[i++];
  float z2 = glcmd.data[i++];
  glMultMatrixf(m);
  glScalef(x1,y1,z1);
  glColor3f(x2,y2,z2);
  t3dDraw3D(glcmd.text, 0, 0, 0.01f);
  glPopMatrix();
}

void CallBackReshapeFunc(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (double)w / (double)h, 0.5, 200.0);
}

void CallBackIdleFunc()
{
}

int getRootWindowSize(int *w, int *h)
{
  Display* pdsp = NULL;
  Window wid = 0;
  XWindowAttributes xwAttr;

  pdsp = XOpenDisplay( NULL );
  if ( !pdsp ) {
  fprintf(stderr, "Failed to open default display.\n");
  return -1;
  }

  wid = DefaultRootWindow( pdsp );
  if ( 0 > (signed) wid ) {
  fprintf(stderr, "Failed to obtain the root windows Id "
      "of the default screen of given display.\n");
  return -2;
  }

  Status ret = XGetWindowAttributes( pdsp, wid, &xwAttr );
  if(ret == 0)
    cerr << "[ERROR]: XGetWindowAttributes returned NULL!" << endl;
  *w = xwAttr.width;
  *h = xwAttr.height;

  XCloseDisplay( pdsp );
  return 0;
}

void drawAxis()
{
  glLineWidth(2.0f);

  glBegin(GL_LINES);
  glColor3f(1.0,0.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(2.0,0.0,0.0);
  glColor3f(0.0,1.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,2.0,0.0);
  glColor3f(0.0,0.0,1.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,0.0,2.0);
  glEnd();

  glPushMatrix();
  glColor3f(1.0,0.0,0.0);
  glTranslatef(2.0,0.0,0.0);
  glRotatef(90,0.0,1.0,0.0);
  glutSolidCone(0.05,0.1,10,10);
  glPopMatrix();
  glPushMatrix();
  glColor3f(0.0,1.0,0.0);
  glTranslatef(0.0,2.0,0.0);
  glRotatef(-90,1.0,0.0,0.0);
  glutSolidCone(0.05,0.1,10,10);
  glPopMatrix();
  glPushMatrix();
  glColor3f(0.0,0.0,1.0);
  glTranslatef(0.0,0.0,2.0);
  glutSolidCone(0.05,0.1,10,10);
  glPopMatrix();
}

GLuint loadTexture(Image* image) {
	GLuint textureId;
	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_RGB,
               image->width, image->height,
               0,
               GL_RGB,
               GL_UNSIGNED_BYTE,

				       image->pixels);
	return textureId;
}

bool readTextureFile(const std::string& filename)
{
  std::vector<std::string> lines;
  std::ifstream readfile (filename.c_str());

  size_t found = filename.find_last_of("/\\");
  std::string folder = filename.substr(0,found+1);

  if (readfile.is_open())
  {

    while ( readfile.good() )
    {
      std::string line;
      std::getline (readfile,line);
      if(line.empty() || line[0] == '#') continue;

      lines.push_back(line);
    }
    readfile.close();

    global.textures.resize(lines.size());

    for(size_t i=0; i<lines.size(); i++)
    {
      global.textures[i].filename.assign(folder);
      global.textures[i].filename.append(lines[i]);
    }

    return(true);
  }

  return(false);
}

void CaptureViewPort()
{
  GLubyte * bits;
  GLint viewport[4];

  glGetIntegerv(GL_VIEWPORT, viewport);

  int w = viewport[2];
  int h = viewport[3];

  bits = new GLubyte[w*3*h];

  glFinish();
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, bits);

  IplImage * capImg = cvCreateImage( cvSize(w,h), IPL_DEPTH_8U, 3);
  for(int i=0; i < h; ++i)
  {
  for(int j=0; j < w; ++j)
  {
   capImg->imageData[i*capImg->widthStep + j*3+0] = (unsigned char)(bits[(h-i-1)*3*w + j*3+0]);
   capImg->imageData[i*capImg->widthStep + j*3+1] = (unsigned char)(bits[(h-i-1)*3*w + j*3+1]);
   capImg->imageData[i*capImg->widthStep + j*3+2] = (unsigned char)(bits[(h-i-1)*3*w + j*3+2]);
  }
  }

  char buff[256];
  sprintf(buff,"/tmp/frame_%04d.jpg",g_frame_counter++);
  if(g_frame_counter>=10000) g_frame_counter = 0;
  cvSaveImage(buff,capImg);
  cvReleaseImage(&capImg);
  delete[] bits;
}
