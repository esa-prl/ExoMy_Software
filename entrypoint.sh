#!/bin/bash
source /opt/ros/melodic/setup.bash
cd /root/exomy_ws
catkin_make
source devel/setup.bash

roslaunch exomy exomy.launch
