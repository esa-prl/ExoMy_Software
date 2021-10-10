#!/bin/bash
if [[ $1 == "config" ]]
then
	cd /root/exomy_ws/src/exomy/scripts
	bash
elif [[ $1 == "autostart" ]]
then
	source /opt/ros/noetic/setup.bash
	cd /root/exomy_ws
	catkin build
	http-server src/exomy/gui -p 8000 &

	source devel/setup.bash
	roslaunch exomy exomy.launch

	bash
elif [[ $1 == "devel" ]]
then
	cd /root/exomy_ws
	source /opt/ros/noetic/setup.bash
	# catkin build
	# source devel/setup.bash
	bash
else
	bash
fi