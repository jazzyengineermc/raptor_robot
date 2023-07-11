#!/usr/bin/bash
#
# Bring up da buggy
#
#########################
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roscore &
sleep 3

roslaunch raptor_robot testing.launch &
# /home/jreide/catkin_ws/src/raptor_robot/src/homebin/rc_mode.sh &
