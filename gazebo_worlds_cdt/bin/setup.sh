#!/bin/bash

echo '
export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/gazebo_worlds_cdt/models:$GAZEBO_MODEL_PATH' >> $HOME/.bashrc

source $HOME/.bashrc