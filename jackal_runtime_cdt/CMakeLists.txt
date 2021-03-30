cmake_minimum_required(VERSION 2.8.3)

project(jackal_runtime_cdt)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS
#    include
  CATKIN_DEPENDS eigen_conversions
)

catkin_python_setup()
catkin_install_python(PROGRAMS python/jackal_to_robot_state.py
                               python/perfect_jackal_to_robot_state.py
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
#  include
  ${catkin_INCLUDE_DIRS}
)

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
