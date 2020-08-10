#!/bin/bash
# Runs a docker container based on the exomy image that
# autostarts every time ExoMy is started

docker run \
    -it \
    -v $(pwd)/ExoMy_Software:/root/exomy_ws/src/exomy \
    -p 8000:8000 \
    -p 8080:8080 \
    -p 9090:9090 \
    --restart always \
    --privileged \
    --name exomy \
    exomy \
    autostart