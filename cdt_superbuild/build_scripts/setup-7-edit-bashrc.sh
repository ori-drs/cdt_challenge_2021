#!/bin/bash


WORKING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../../ && pwd )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../ && pwd )"

echo '

source /opt/ros/melodic/setup.bash
export CATKIN_WORKSPACE='"${WORKING_DIR}"'/catkin_ws
source $CATKIN_WORKSPACE/devel/setup.bash' >> $HOME/.bashrc

if [ $? -ne 0 ]; then
    echo "Cannot edit bashrc"
    exit 8
fi
