#! /usr/bin/bash
sudo apt-get install -y python3-pip
pip3 install eventlet==0.33.3
pip3 install Flask==1.1.1
pip3 install Flask-SocketIO==4.1.0
pip3 install python-socketio==4.2.0
pip3 install python-engineio==3.13.0
pip3 install greenlet==1.0.0
pip3 install gevent==21.1.2
pip3 install gevent-websocket==0.10.1
pip3 install Jinja2==3.0.3
pip3 install itsdangerous==2.0.1
pip3 install werkzeug==2.0.3
 
 
pip3 install attrdict
pip3 install numpy
pip3 install pillow
pip3 install opencv-contrib-python

sudo apt-get install -y ros-foxy-tf-transformations
sudo pip3 install transforms3d
sudo apt-get install -y ros-$ROS_DISTRO-rviz-imu-plugin

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp
sudo apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox