#!/bin/bash
cd /home/jreide/catkin_ws
source devel/setup.bash

rostopic pub -l /move_base/cancel actionlib_msgs/GoalID -- {} &

echo "done"
