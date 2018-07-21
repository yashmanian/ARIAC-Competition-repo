cmake_minimum_required(VERSION 2.8.3)
project(competition)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  rospy
  sensor_msgs
  std_srvs
  trajectory_msgs
  std_msgs
  message_generation
  geometry_msgs

  moveit_ros_planning_interface
  moveit_ros_planning_interface
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
)


########################
## Message Generation ##
########################
add_message_files(
  FILES
  manager.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(competition_node src/manager.cpp src/main.cpp)
add_dependencies(competition_node  competition_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(competition_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
# install(TARGETS competition_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES config/workcell_conf.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
