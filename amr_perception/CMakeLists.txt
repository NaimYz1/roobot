cmake_minimum_required(VERSION 2.8.3)
project(amr_perception)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  src/vfh_plus_node.py
  src/vfh_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
