#!/bin/bash


WORKING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../../ && pwd )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../ && pwd )"

echo '

source /opt/ros/melodic/setup.bash
export CATKIN_WORKSPACE='"${WORKING_DIR}"'/catkin_ws
source $CATKIN_WORKSPACE/devel/setup.bash
export GAZEBO_MODEL_PATH=${CATKIN_WORKSPACE}/src/cdt_challenge_2021/:${CATKIN_WORKSPACE}/src/cdt_challenge_2021/gazebo_worlds_cdt:${CATKIN_WORKSPACE}/src/cdt_challenge_2021/gazebo_worlds_cdt/models:${CATKIN_WORKSPACE}/src/aims_cdt_week/aims_jackal_sim/models:$GAZEBO_MODEL_PATH
export PYTHONPATH=${CATKIN_WORKSPACE}/src/aims_cdt_week/pf_localisation/src:$PYTHONPATH' >> $HOME/.bashrc

if [ $? -ne 0 ]; then
    echo "Cannot edit bashrc"
    exit 8
fi
