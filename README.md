# PROMTS (PRinciple Of Minimum Translation Search)
## A Principle of Minimum Translation Search Approach for Object Pose Refinement

This repository is the implementation of the methodology proposed in the following
paper,

@ARTICLE{Mojtahedzadeh:IROS:2015,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;AUTHOR = {Mojtahedzadeh, Rasoul and Lilienthal, Achim J.},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;TITLE = {A Principle of Minimum Translation Search Approach for Object Pose Refinement},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;BOOKTITLE = {In Proc. of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;YEAR = {2015},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;NOTE = {to appear},  
}

to autonomously resolve the inter-penetrations between adjacent objects models due to error in
the estimated poses of the objects.

# Paper Abstract:
The state-of-the-art object pose estimation approaches represent the set of detected poses together
with corresponding uncertainty. The inaccurate noisy poses may result in a configuration of overlapping
objects especially in cluttered environments. Under a rigid body assumption the inter-penetrations between
pairs of objects are geometrically inconsistent. In this paper, we propose the principle of minimum
translation search, PROMTS, to find an inter-penetration-free configuration of the initially detected objects.
The target application is to automate the task of unloading shipping containers, where a geometrically consistent
configuration of objects is required for high level reasoning and manipulation. We find that the proposed approach
to resolve geometrical inconsistencies improves the overall pose estimation accuracy. We examine the utility of
two selected search methods: A-star and Depth-Limited search. The performance of the search algorithms are tested
on data sets generated in simulation and from real-world scenarios. The results show overall improvement of the 
estimated poses and suggest that depth-limited search presents the best overall performance.

## Implementation
The software is implemented in C/C++ and under ROS (Robot Operation System) and tested with ROS Hydro.
The software uses the fast implementation of 3D convex hull estimation algorithm in Bullet Physics engine 
LinearMath.h header file. Thus, you have to install it on your linux distribution. For example, if your linux
distribution is Ubuntu, you can install it using the following command line,

`sudo apt-get install libbullet libbullet-dev`

The software also requires Eigen library,

`sudo apt-get install libeigen3-dev`

## ROS packages
There are following ROS packages included in this software:
* **rasoul\_promts\_pkg**
  This is the main package of the software. It includes sample codes for demonstration of how to use the software.
* **rasoul\_geometry\_pkg**
  This package implements necessary computational geometry functions for being used with the main package.
* **rasoul\_collision\_detection\_pkg**
  This package implements Separating Axis Theorem (SAT) in 3-dimentional space in order to compute Depth of
  Penetration (DOP) and Minimum Translation Vector (MTV) between two convex polyhedra.
* **rasoul\_visualizer\_pkg**
  This package implements a visualizer using OpenGL in order to visualize a configuration of objects and animate
  the found resolving solution.
* **rasoul\_common\_pkg**
  This package implements a set of fucntions and classes being used by other packages.

## Run demo
There are two demo codes,
* **DemoPROMTSAstar.cpp;**
* **DemoPROMTSDLS.cpp**
that you can find in th following path

`rasoul_promts_pkg/demo/`

and both demo source codes are fed with a sample noisy configuration of objects data which can be found in the following path

`rasoul_promts_pkg/examples/example1_noisy.cfg`

There are two launch files example1_Astar.launch and example1_DLS.launch in the following path

`rasoul_promts_pkg/launch/`

Runing the launch files, they will bring up the visualizer and execute the demo PROMTS algorithms using A* or Depth-Limited Search
correspondingly.

## Use PROMTS
The implementation assumes that there exists an object detection and pose estimation that estimated the poses of objects. The models
of the objects are assumed to be convex polyhedrons. 

Following steps show how to use PROMTS,
1. Use the CGeometryPolyhedra class (rasoul_geometry_pkg) to compute a convex polyhedron for the objects.
2. Fill in a Eigen::Transform<Real,3,Eigen::Affine> vector with the noisy poses.
3. Fill in a std::vector<int> vector of objects' IDS.
4. Choose between a search algorithm for PROMTS
   - Use the refinePoses_AStarSearch function (rasoul_promts_pkg) to get an inter-penetration free set of poses using A* search algorithm.
   - Use the refinePoses_DLSearch function (rasoul_promts_pkg) to get an inter-penetration free set of poses using DLSearch algorithm.
