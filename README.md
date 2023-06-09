# raptor_robot
My adventures in building a usefull robot base platform. Using 2 hoverboard motors, ODrive v3.6 (firmware 0.5.5), Teensy 4.0, Jetson Nano 4gb (B01), and a third wheel for caster as this won't be a balancing robot. A few other sensors to round out SLAM include 2 RPLIDAR A1's using ira_laser_tools to fuse them for 360 degree view around the robot with a 5gal bucket in the middle and a RealSense D435i that will help with 3d mapping and provide info to local costmap to aide in obsticle avoidance. For "FPV" operations and "line following" there is a raspberry pi v2 camera up front and for "lead the way" functions there is a rear facing webcam. With the webcam providing a microphone and built in speakers on the monitor, getting a voice interface is very high on the priority list. Please Check Wiki for more details.

ROS packages needed-

  ros-noetic-rplidar-ros
  ros-noetic-hector-slam
  ros-noetic-ira-laser-tools
  ros-noetic-usb-cam
  ros-noetic-realsense2-camera
  ros-noetic-navigation
