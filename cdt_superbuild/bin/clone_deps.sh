#!/bin/bash
# Jackal packages


# Ouster packages
git clone git@github.com:ori-drs/ouster_example.git

# Elevation mapping
git clone --branch release git@github.com:ANYbotics/elevation_mapping.git

# Kindr
git clone --branch release git@github.com:ANYbotics/kindr.git

# Kindr ROS
git clone --branch master git@github.com:ANYbotics/kindr_ros.git

# Velodyne simulation
git clone --branch 1.0.12 https://bitbucket.org/DataspeedInc/velodyne_simulator.git