cmake_minimum_required(VERSION 2.8.3)
project(amr_planning)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  src/planner_node.py
  src/prm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
