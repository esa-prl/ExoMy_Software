#!/bin/bash
if [ $1 == "config" ]
then
	bash
else
	source /opt/ros/melodic/setup.bash
	cd /root/exomy_ws
	catkin_make
	http-server src/exomy/gui -p 8000 &

	source devel/setup.bash
	roslaunch exomy exomy.launch

	bash
fi

