#!/bin/bash
# Runs a docker container based on the exomy image that
# starts the motor configuration script

docker run \
    -it \
    -v $(pwd)/ExoMy_Software:/root/exomy_ws/src/exomy \
    --rm \
    -p 8000:8000 \
    -p 8080:8080 \
    -p 9090:9090 \
    --privileged \
    --name exomy_motor_config\
    exomy \
    config 