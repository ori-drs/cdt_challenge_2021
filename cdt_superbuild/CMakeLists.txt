cmake_minimum_required(VERSION 2.8.3)
project(cdt_superbuild)

find_package(catkin REQUIRED
  cdt_msgs
  cdt_rqt
  cdt_visualizations
  gazebo_worlds_cdt
  jackal_description_cdt
  jackal_runtime_cdt
  elevation_map_filter_cdt
  object_detector_cdt
  world_modelling_cdt
  world_explorer_cdt
  position_controller_cdt
)

catkin_package(
  CATKIN_DEPENDS 
)

install(DIRECTORY bin# install launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
