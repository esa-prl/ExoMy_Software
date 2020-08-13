#!/bin/bash
# run_exomy- A script to run containers for dedicated functions of exomy

help_text="Usage: "$0" [MODE] [OPTIONS]
    A script to run ExoMy in different configurations
    Options:
        -a, --autostart     Toggles autostart mode on or off
        -c, --config        Runs the motor configuration of ExoMy
        -d, --devel         Runs the development mode to change some code of ExoMy 
        -h, --help          Shows this text
"

### Main
# Initialize parameters 
container_name="exomy"
image_name="exomy"

# Process parameters
if [ "$1" != "" ]; then
    case $1 in
        -a | --autostart)       
                                container_name="${container_name}_autostart"
                                start_command="autostart"
                                options="--restart always"
                                 
                                ;;
        -c | --config)          
                                container_name="${container_name}_config"
                                start_command="config"
                                options="--rm"
                                ;;
        -d | --devel)           
                                container_name="${container_name}_devel"
                                start_command="devel"
                                options="--restart always"
                                ;;  
        -h | --help )           echo "$help_text"
                                exit
                                ;;
        * )                     echo "ERROR: Not a valid mode!"
                                echo "$help_text"
                                exit 1
    esac
else
    echo "ERROR: You need to specify a mode!"
    echo "$help_text"
    exit
fi

# Check for image and build from Dockerfile if not accessible
result=$( docker images -q $image_name )

if [ ! -n "$result" ]; then
    directory=$( dirname "$0" )

    echo "Docker image not built yet. Build from Dockerfile. "
    echo ${directory}
    docker build -t $image_name $directory
else
    echo "Use existing docker image: "$image_name""
fi

# Run docker container
docker run \
    -it \
    -v $(pwd)/ExoMy_Software:/root/exomy_ws/src/exomy \
    -p 8000:8000 \
    -p 8080:8080 \
    -p 9090:9090 \
    --privileged \
    ${options} \
    --name "${container_name}" \
    "${image_name}" \
    "${start_command}"