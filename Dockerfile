# Base image
FROM ros:melodic 

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic tools
RUN apt-get update && \
    apt-get install vim git tmux wget curl python-pip -y


# ZSH Installation
RUN apt-get update && \
    apt-get install zsh -y
ENV ZSH_THEME avit
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
RUN echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc

# Install additional ros packages
RUN apt-get install ros-melodic-rosbridge-server ros-melodic-joy -y
RUN pip install adafruit-pca9685


# Install packages for web application
RUN curl -sL https://deb.nodesource.com/setup_12.x | bash -
RUN apt-get update && \
    apt-get install nodejs -y
RUN npm install http-server -g

# Install packages for camera use
RUN apt-get update && \
    apt-get install ros-melodic-web-video-server ros-melodic-usb-cam -y



# Create Ros workspace
RUN mkdir -p /root/exomy_ws/src

# Start zsh
CMD [ "zsh" ]
