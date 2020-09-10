#!/bin/bash
if [[ $1 == "config" ]]
then
	cd /root/exomy_ws/src/exomy/scripts
	zsh	
elif [[ $1 == "autostart" ]]
then
	source /opt/ros/foxy/setup.bash
	cd /root/exomy_ws
	colcon build	
	http-server src/exomy/gui -p 8000 &

	source install/setup.zsh
	roslaunch exomy exomy.launch

	zsh
elif [[ $1 == "devel" ]]
then
	cd /root/exomy_ws
	source /opt/ros/foxy/setup.bash
	catkin_make
	source install/setup.bash
	zsh	
else
	zsh
fi
