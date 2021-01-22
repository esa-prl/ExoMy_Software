# Base image
FROM ros:foxy

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic tools
RUN apt-get update && \
    apt-get install vim nano git tmux wget curl python3-pip net-tools iputils-ping zsh fonts-powerline -y

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

ENV NPM_CONFIG_PREFIX=/home/exomy/.npm-global
ENV PATH=$PATH:/home/exomy/.npm-global/bin
RUN npm install http-server -g

# Create Ros workspace
ENV EXOMY_WS=/home/exomy/exomy_ws
RUN mkdir -p $EXOMY_WS/src
WORKDIR /root


RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

ADD entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "bash" ]
