cmake_minimum_required(VERSION 3.0.2)
project(move_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MoveArm.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS  rospy )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)