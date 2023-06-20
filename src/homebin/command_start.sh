#!/usr/bin/bash
#
# Bring up da buggy
#
#########################
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://nano.local:11311

# roscore &
# sleep 3

roslaunch raptor_robot bringup_command.launch