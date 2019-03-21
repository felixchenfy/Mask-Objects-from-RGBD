
Intel® RealSense, D435

## Properties

Depth output resolution: 1280 x 720   
Min depth: 0.2m~10m  


## General Tutorials

* ROS wiki::  
http://wiki.ros.org/realsense2_camera       


* Specifications: https://click.intel.com/intelr-realsensetm-depth-camera-d435.html


# Specific tutorials and commands

First step !!!
install dependencies:
  https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

## Run its own API
$ realsense-viewer

## Run in cpp
A getting started example with CMakeLists
https://github.com/zivsha/librealsense/tree/getting_started_example/examples/getting-started

Work with opencv
https://github.com/IntelRealSense/librealsense/blob/master/doc/stepbystep/getting_started_with_openCV.md

many c++ examples
https://github.com/IntelRealSense/librealsense/tree/master/examples

## Run in python (not working)
$ pip install pyrealsense2

https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
"does not work with the RealSense SDK 2.0"

## Run in ROS
See: https://github.com/intel-ros/realsense/#installation-instructions

downoad a ROS package and copy to ~/catkin_ws/src and catkin_make
  https://github.com/intel-ros/realsense/releases
Then:

$ roslaunch realsense2_camera rs_camera.launch
Output: rgb/depth/infra image

$ roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true
Output: point cloud (but i'm not sure which frame it is in, if directly sub to this topic)

$ roslaunch realsense2_camera rs_camera.launch filters:=colorizer align_depth:=true
Colored depth image.

## Others
About rectify,
https://github.com/IntelRealSense/librealsense/issues/91
It says, when allign depth and color, they are all rectified.
Besides, actually, if we want the depth by computing from two infra images, we need rectification.

# Useful info

Enabling post processing filters.
realsense2_camera includes some built in post processing filters:
colorizer - creates an RGB image instead of depth image. Used to visualize the depth image.
spatial - filter the depth image spatially.
temporal - filter the depth image temporally.
pointcloud - it is now possible to enable point cloud with the same command as any other post processing filter.

The colorizer filter replaces the image in the topic: //depth/image_rect_raw. The spatial and temporal filters affect the depth image and all that is derived from it, i.e. pointcloud and colorizer.

to activate the filters, use the argument "filters" and deperate them with a comma:

roslaunch realsense2_camera rs_camera.launch filters:=temporal,spatial,pointcloud
  