cmake_minimum_required(VERSION 2.8.3)

project(cdt_visualizations)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  tf_conversions
  roscpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf
  cdt_msgs
  grid_map_ros
  grid_map_core
  grid_map_cv
  grid_map_msgs
)

catkin_package(
  INCLUDE_DIRS
#    include
  CATKIN_DEPENDS roscpp
                eigen_conversions
                geometry_msgs
                std_msgs
                tf2
                tf2_ros
                tf
                cdt_msgs
                grid_map_ros
                grid_map_core
                grid_map_cv
                grid_map_msgs
)

catkin_python_setup()
catkin_install_python(PROGRAMS python/publish_visualizations.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs  # Or other packages containing msgs
#   actionlib_msgs
#)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}
            src/cdt_visualizations/explored_space_vis.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

# Create node
add_executable(explored_space_vis_node  src/explored_space_node.cpp)
target_link_libraries(explored_space_vis_node ${PROJECT_NAME})
#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
