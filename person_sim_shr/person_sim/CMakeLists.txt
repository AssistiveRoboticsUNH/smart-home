cmake_minimum_required(VERSION 2.8.3)
project(person_sim)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  spawn_robot_tools_pkg
)

catkin_package(
  CATKIN_DEPENDS rospy gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
