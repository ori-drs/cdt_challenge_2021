#!/bin/bash

WORKING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../../ && pwd )"

function quit {
    echo "*** Build finished at $(date) ***"
}

trap "quit" EXIT

if [ $# -eq 1 ]; then
    if [ "$1" = "--nsm" ]; then
        echo "Will build nsm"
    fi
fi

# Install deps
echo "Installing dependencies..."

echo "Setting up build environment"
bash "$(dirname "${BASH_SOURCE[0]}" )/setup-1-build-environment.sh"
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ####################################################################
    Failed to setup the build environment (setup-1-build-environment.sh)
    ####################################################################
    "
    exit $exit_status
fi

echo "Adding ROS package sources"
bash "$(dirname "${BASH_SOURCE[0]}" )/setup-2-install-ros.sh"
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ################################################################################
    Failed to add ROS repositories (setup-2-install-ros.sh)
    ################################################################################
    "
    exit $exit_status
fi

echo "Installing ROS packages"
bash "$(dirname "${BASH_SOURCE[0]}" )/setup-3-cdt-packages.sh"
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ########################################################################
    Failed to install the CDT packages (setup-3-cdt-packages.sh)
    ########################################################################
    "
    exit $exit_status
fi
echo "
*************************
DONE GETTING DEPENDENCIES
*************************

============================================================================
"
echo "

****************
BEGIN CODE SETUP
****************

"

echo "Getting the submodules"
bash "$(dirname "${BASH_SOURCE[0]}" )/setup-4-update-submodules.sh"
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ###########################################################
    Failed to get the submodules (setup-4-update-submodules.sh)
    ###########################################################
    "
    exit $exit_status
fi

echo "
*****************
INIT WORKSPACE
*****************
$(date)
"


bash "$(dirname "${BASH_SOURCE[0]}" )/setup-5-init-workspace.sh" $1
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ###############################################
    Failed to build the code (setup-5-init-workspace.sh)
    ###############################################
    "
    exit $exit_status
fi

echo "
*****************
BUILDING THE CODE
*****************
$(date)
"


bash "$(dirname "${BASH_SOURCE[0]}" )/setup-6-build.sh" $1
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ###############################################
    Failed to build the code (setup-6-build.sh)
    ###############################################
    "
    exit $exit_status
fi

echo "
***************
EDITING BASHRC
***************
$(date)
"


bash "$(dirname "${BASH_SOURCE[0]}" )/setup-7-edit-bashrc.sh"
exit_status=$?
if [ $exit_status -ne 0 ]; then
    echo "
    ################################################
    Failed to edit bashrc (setup-7-update-bashrc.sh)
    ################################################
    "
    exit $exit_status
fi
