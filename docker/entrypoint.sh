#!/bin/bash
if [ $1 == "config" ]
then
	bash
elif [ $1 == "autostart" ]
	source /opt/ros/melodic/setup.bash
	cd /root/exomy_ws
	catkin_make
	http-server src/exomy/gui -p 8000 &

	source devel/setup.bash
	roslaunch exomy exomy.launch

	bash
elif [ $1 == "ros" ]
	cd /root/exomy_ws
	catkin_make
	source devel/setup.bash
	bash
else
	bash
fi

