# Base image
FROM ros:foxy

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic tools
RUN apt-get update && \
    apt-get install vim nano git tmux wget curl python3-pip net-tools iputils-ping zsh fonts-powerline -y

# Create and switch to non-root user in sudoers group
RUN adduser --disabled-password --gecos '' exomy --shell /bin/zsh &&\
    adduser exomy sudo &&\
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER exomy

# OH-MY-ZSH Installation
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh)" &&\
    sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/' ~/.zshrc &&\
    echo "source /opt/ros/foxy/setup.zsh" >> ~/.zshrc

# Install additional ros packages
RUN sudo apt-get update && \
    sudo apt-get install ros-foxy-joy -y
RUN pip3 install adafruit-pca9685


# Install packages for web application
RUN curl -sL https://deb.nodesource.com/setup_12.x | sudo bash -
RUN sudo apt-get update && \
    sudo apt-get install nodejs -y
RUN sudo npm install http-server -g

# Install packages for camera use
# RUN apt-get update && \
# apt-get install ros-foxy-web-video-server ros-foxy-usb-cam -y

# Add ros sourcing to bashrc
# RUN echo ". /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Create Ros workspace
ENV EXOMY_WS=/home/exomy/exomy_ws
RUN mkdir -p $EXOMY_WS/src

WORKDIR /home/exomy

ADD entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
