cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_waypoints)
find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  actionlib_msgs
  geometry_msgs
  message_generation
)
catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################
add_action_files(
  FILES
  WaypointAction.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS
	rospy
    message_runtime
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tortoisebot_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Enable rostest
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/waypoints_test.test)
endif()

catkin_add_nosetests(test/waypoints_test_ros_as.py)