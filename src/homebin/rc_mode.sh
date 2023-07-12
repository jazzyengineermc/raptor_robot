#!/bin/bash
cd /home/jreide/catkin_ws
source devel/setup.bash

rostopic pub raptor/control std_msgs/String rc
