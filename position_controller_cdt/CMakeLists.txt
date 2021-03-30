cmake_minimum_required(VERSION 2.8.3)

project(position_controller_cdt)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp
                  eigen_conversions
                  geometry_msgs
                  std_msgs
                  tf2
                  tf2_ros
                  tf
)

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
            src/position_controller_cdt/position_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

# Create node
add_executable(${PROJECT_NAME}_node  src/position_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})



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
