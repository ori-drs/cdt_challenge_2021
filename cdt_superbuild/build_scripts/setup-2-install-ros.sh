#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Try up to 10 times to get the key 
for i in {1..10}; do
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    return_value=$?
    if [ $return_value -ne 0 ] && [ $i -ne 10 ]; then
        echo "Addition of key failed, retry in 5s"
        sleep 5s
    elif [ $return_value -ne 0 ] && [ $i -eq 10 ]; then
        echo "Retrival of ROS packages failed"
        exit 2
    else
        break	
	fi
done

sudo apt update
sudo apt install -y ros-melodic-desktop-full
if [ $? -ne 0 ]; then
    echo "Installation of ros-melodic-desktop-full failed"
    exit 2
fi

#initialize rosdep
sudo apt install python-rosdep
sudo rosdep init
rosdep update

sudo apt-get update

