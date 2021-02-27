cmake_minimum_required(VERSION 2.8.3)
project(cdt_superbuild)

find_package(catkin REQUIRED
  cdt_msgs
  gazebo_worlds_cdt
  jackal_description_cdt
  jackal_runtime_cdt
  object_detector_cdt
  world_modelling_cdt
  world_explorer_cdt
)

catkin_package(
  CATKIN_DEPENDS 
)

install(DIRECTORY bin# install launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
