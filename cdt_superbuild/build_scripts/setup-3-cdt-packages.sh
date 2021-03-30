#!/bin/bash

#Install all the packages listed after the while loop
exit_status=0
while read -r package; do
    if [[ ! $package == "#"* ]] && [[ ! $package == "" ]]; then
        sudo apt-get install -y "$package"
        exit_status=$?
        if [ $exit_status -ne 0 ]; then
            echo "Installation of $package failed"
            break
        fi
    fi
done <<< "# Add packages to install below here
# Lines beginning with # and empty lines are ignored

python-catkin-tools

# ROS
ros-melodic-ros-control
ros-melodic-serial
ros-melodic-octomap-ros
ros-melodic-grid-map
ros-melodic-grid-map-ros
ros-melodic-rqt-multiplot
ros-melodic-hector-models
ros-melodic-grid-map-pcl
ros-melodic-octomap-ros
ros-melodic-gazebo-ros
ros-melodic-gazebo-ros-control
ros-melodic-robot-controllers
ros-melodic-controller-interface
ros-melodic-joint-state-controller
ros-melodic-diff-drive-controller
ros-melodic-velocity-controllers
ros-melodic-effort-controllers
ros-melodic-position-controllers
ros-melodic-twist-mux
ros-melodic-ros-control
ros-melodic-mongodb-store-msgs
ros-melodic-mongodb-store

# Jackal Packages
ros-melodic-jackal-simulator
ros-melodic-jackal-desktop
ros-melodic-jackal-navigation

# Others
ros-melodic-grid-map-filters
ros-melodic-grid-map-loader
ros-melodic-grid-map-pcl
ros-melodic-grid-map-visualization
ros-melodic-grid-map-octomap
ros-melodic-grid-map-rviz-plugin
ros-melodic-grid-map-sdf
ros-melodic-grid-map-cv
ros-melodic-velodyne-msgs
ros-melodic-velodyne-pointcloud
ros-melodic-velodyne-gazebo-plugins
ros-melodic-ompl
"

if [ $exit_status -ne 0 ]; then
    exit 4
fi
