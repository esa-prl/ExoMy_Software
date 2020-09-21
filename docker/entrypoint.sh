#!/bin/bash
set -e

if [[ $1 == "config" ]]
then
	cd "/home/exomy/exomy_ws/src/exomy/scripts"
	bash
elif [[ $1 == "autostart" ]]
then
	cd "/home/exomy/exomy_ws"

	source "/opt/ros/foxy/setup.bash"
	colcon build
	source "/home/exomy/exomy_ws/install/setup.bash"
	
	cd "/home/exomy/exomy_ws/src/exomy_gui"

	node node_modules/ros2-web-bridge/bin/rosbridge.js &
	http-server -p 8000 &

	# Sleep is needed to first print output and yjem start bash
	sleep 1
	
	cd "/home/exomy/exomy_ws"
	bash
elif [[ $1 == "devel" ]]
then
	cd "/home/exomy/exomy_ws"
	source "/opt/ros/foxy/setup.bash"
	colcon build
	source "/home/exomy/exomy_ws/install/setup.bash"
	
	bash
else
	bash
fi
