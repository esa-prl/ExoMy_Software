# Base image
FROM ros:melodic 

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update the GPG key for packages.ros.org first

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt update

# Basic tools
RUN apt-get update && \
    apt-get install vim nano git tmux wget curl python-pip net-tools iputils-ping  -y

# Install additional ros packages
RUN apt-get update && apt-get install ros-melodic-rosbridge-server ros-melodic-joy -y
RUN pip install adafruit-pca9685


# Install packages for web application
RUN curl -sL https://deb.nodesource.com/setup_12.x | bash -
RUN apt-get update && \
    apt-get install nodejs -y
RUN npm install http-server -g

# Install packages for camera use
RUN apt-get update && \
    apt-get install ros-melodic-web-video-server ros-melodic-usb-cam -y

# Add ros sourcing to bashrc
RUN echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Create Ros workspace
ENV EXOMY_WS=/root/exomy_ws
RUN mkdir -p $EXOMY_WS/src

WORKDIR /root

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
