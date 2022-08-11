#!/bin/bash

dependencies=(
    git
    cmake
    python3-rosdep
    build-essential
    ros-noetic-catkin
    python3-catkin-tools 
)

apt update
apt install -y --no-install-recommends ${dependencies[@]}

mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/
git clone --depth 1 https://github.com/tonykolomeytsev/camera_handle.git src/camera_handle

source /opt/ros/noetic/setup.bash
catkin init
catkin_make camera_handle

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source /root/catkin_ws/devel/setup.bash' >> ~/.bashrc
