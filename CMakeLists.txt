cmake_minimum_required(VERSION 3.0.2)
project(ares_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  tf2_ros
  pcl_ros
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

find_package(PCL REQUIRED)

add_message_files(
  FILES
  GoalSite.msg
  MotorCommand.msg
  MotorFeedback.msg
  WheelCommandArray.msg
)

add_service_files(
  FILES
  CheckPath.srv
  GetPath.srv
)

add_action_files(DIRECTORY action FILES FollowPath.action)

generate_messages(
  DEPENDENCIES
  actionlib_msgs nav_msgs std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PathController.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ares_control
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ares_control src/main.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ares_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} tmotor)

install(FILES
  launch/main.launch
  launch/joy.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)