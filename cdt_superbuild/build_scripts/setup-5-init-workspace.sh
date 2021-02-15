#!/bin/bash


WORKING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../../ && pwd )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../ && pwd )"

echo "Creating catkin workspace directory..."
mkdir -p "${WORKING_DIR}/catkin_ws/src"
if [ $? -ne 0 ]; then
    echo "Creation of catkin directory failed"
    exit 6
fi

source /opt/ros/melodic/setup.bash

export SHELL=/bin/bash

pushd "${WORKING_DIR}/catkin_ws"
echo "Initializing catkin workspace"
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

ln -s "$DIR"/* "$WORKING_DIR/catkin_ws/src"

