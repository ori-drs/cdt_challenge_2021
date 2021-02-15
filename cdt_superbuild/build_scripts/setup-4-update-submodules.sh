#!/bin/bash

exit_status=0
CLONE_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../bin/ && pwd )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../ && pwd )"
pushd "${DIR}"

#Clone all the repositories stored inside clone_deps.sh 
DEPS=`grep "git clone" $CLONE_SCRIPT_DIR/clone_deps.sh`

while read -r line ; do
#    echo "Processing $line"
    eval "$line"
    exit_status=$?
    if [ $exit_status -ne 0 ]; then
    	echo "Creation of catkin package failed"
    	break
    fi
        # this exits a subshell, not the whole script
        # the exit essentially functions as a break statement
        # but sets the variable $? as well
done <<< "$DEPS"

#exit with an error if git clone failed
if [ $exit_status -ne 0 ]; then
    exit 5
else
    popd
fi
