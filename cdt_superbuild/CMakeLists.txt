cmake_minimum_required(VERSION 2.8.3)
project(cdt_superbuild)

find_package(catkin REQUIRED
)

catkin_package(
)

install(DIRECTORY bin# install launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
