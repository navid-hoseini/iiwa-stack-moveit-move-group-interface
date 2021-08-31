cmake_minimum_required(VERSION 3.0.2)
project(iiwa_move_group_py_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  moveit_ros_planning_interface
  message_generation
)

# Declare the message files to be built
add_service_files(
  FILES
  AddTwoInts.srv
  MoveitInterface.srv
  AddTwoFloatArrays.srv
)

# Warning: generate_messages() must be called after add_service_files()
#
# Calling generate_messages() is common for msg and srv
generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# According to "https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29"
# makes sure the python script gets installed properly, and uses the right python interpreter:
#
catkin_install_python(PROGRAMS
 scripts/srv_demo/srv_service_demo.py
 scripts/srv_demo/srv_client_demo.py
 scripts/srv_demo/srv_service_array_demo.py
 scripts/srv_demo/srv_client_array_demo.py
 scripts/interface_to_moveit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

