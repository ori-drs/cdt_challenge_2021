#!/bin/bash

echo "Installation of dependencies"
sudo apt-get update -y

sudo apt-get autoremove -y

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
# ** git related things **
git
gitk
git-gui

# **Java**
openjdk-8-jre
openjdk-8-jdk

# **Boost and essential dependencies**
build-essential
cmake
debhelper
freeglut3-dev
libboost-filesystem-dev
libboost-iostreams-dev
libboost-program-options-dev
libboost-random-dev
libboost-regex-dev
libboost-signals-dev
libboost-system-dev
libboost-thread-dev
libcurl4-openssl-dev
libfreeimage-dev
libglew-dev
libltdl-dev
libgsl0-dev
libportmidi-dev
mercurial
libv4l-dev
python-numpy
python-scipy
swig
libsuitesparse-dev
libeigen3-dev
libsdl1.2-dev
doxygen
graphviz
libusb-1.0
python-yaml
libusb-1.0-0-dev
libjpeg-dev
libgtk2.0-dev
qt4-designer
qt4-qmake
cmake
libtinyxml-dev
libsuitesparse-dev
libeigen3-dev
libsdl1.2-dev
doxygen
graphviz
libqhull-dev
python-lxml
python-pyparsing
python-matplotlib
python-pip
libyaml-cpp-dev
libgtkmm-2.4-dev
libqwt-qt5-dev
qtscript5-dev
qtmultimedia5-dev
qtbase5-private-dev

libncurses5-dev
lsb-release
gnupg2
ca-certificates
curl
libglfw3-dev
libtclap-dev

libzbar-dev
python-pymongo
mongodb
scons
"

sudo apt update

# Python
sudo pip install virtualenv
sudo pip install pyttsx

if [ $exit_status -ne 0 ]; then
    exit 1
fi
