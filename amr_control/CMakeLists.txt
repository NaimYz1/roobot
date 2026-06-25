cmake_minimum_required(VERSION 2.8.3)
project(amr_control)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs tf)

catkin_package()

catkin_install_python(PROGRAMS
  src/pp_controller_node.py
  src/pure_pursuit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
