cmake_minimum_required(VERSION 3.0.2)
project(cdt_rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  cdt_msgs
)

catkin_package(
  CATKIN_DEPENDS cdt_msgs
)

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/cdt_rqt
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/cdt_rqt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
