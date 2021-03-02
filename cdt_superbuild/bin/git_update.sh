# Originally from bitbucket.org/leggedrobotics/helper_scripts.git
# This function goes through all folders in your current directory and performs git pull.

#!/bin/bash

# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_CODE="\033[0m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"

# Function to reset the terminal colors
function color_reset
{
  echo -ne "\033[0m"
}

# Function to update git repository (git pull)
function update
{
  F=${1}
  FULL_PATH=$PWD${F:1:${#F}}
  cd ${FULL_PATH}
  MSG="$(git -c color.ui=always pull 2>&1)"
  # git -c color.ui=always pull > MSG

  # mutex lock
  lockdir=/tmp/myscript.lock # lock name has to be the same as in hg_update_faster.sh
  mkdir "$lockdir" 2>/dev/null
  while [ $? -ne 0 ]; do mkdir "$lockdir" 2>/dev/null; done
  # print git pull result
  printf "${COLOR_BOLD}Updated:${COLOR_RESET} ${COLOR_ITEM}${1}${COLOR_RESET}\n${MSG}\n\n"
  # mutex unlock
  rm -rf $lockdir
}

SCRIPT_FOLDER=$(dirname $0)

# Get target folder
if [ $# -eq 1 ]; then
  TARGET=$1
else
  TARGET="."
fi

# Read the list of folders
echo ""
echo -e "${COLOR_INFO}List of repositories:${COLOR_RESET}"
echo ""
for FOLDER in ${TARGET}/*/; do
  if ( cd ${FOLDER} && git rev-parse --git-dir > /dev/null 2>&1 ); then # check if folder is part of a git repo
    update ${FOLDER} &
  fi
done
wait
echo ""
color_reset
