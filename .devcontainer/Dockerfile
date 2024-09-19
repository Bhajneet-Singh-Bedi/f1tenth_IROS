FROM osrf/ros:foxy-desktop
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

RUN sudo usermod --append --groups video $USERNAME


RUN sudo apt install -y git
RUN rosdep update

RUN sudo apt-get install -y python3-pip &&\
    pip3 install eventlet==0.33.3 &&\
    pip3 install Flask==1.1.1 &&\
    pip3 install Flask-SocketIO==4.1.0 &&\
    pip3 install python-socketio==4.2.0 &&\
    pip3 install python-engineio==3.13.0 &&\
    pip3 install greenlet==1.0.0 &&\
    pip3 install gevent==21.1.2 &&\
    pip3 install gevent-websocket==0.10.1 &&\
    pip3 install Jinja2==3.0.3 &&\
    pip3 install itsdangerous==2.0.1 &&\
    pip3 install werkzeug==2.0.3 &&\
    pip3 install attrdict &&\
    pip3 install numpy &&\
    pip3 install pillow &&\
    pip3 install opencv-contrib-python &&\
    sudo apt-get install -y ros-foxy-tf-transformations &&\
    sudo pip3 install transforms3d &&\
    sudo apt-get install -y ros-$ROS_DISTRO-rviz-imu-plugin &&\
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc &&\
    sudo apt install -y ros-foxy-rmw-cyclonedds-cpp &&\
    sudo apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.

CMD ["/bin/bash"]