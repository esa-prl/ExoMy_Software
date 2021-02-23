#!/bin/bash

node node_modules/ros2-web-bridge/bin/rosbridge.js &
http-server -p 8000 && fg