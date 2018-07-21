cmake_minimum_required(VERSION 2.8.3)
project(arm_controller)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

## Declare a C++ executable
add_executable(arm_node src/arm_node.cpp src/armController.cpp)
add_dependencies(arm_node arm_controller_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(FILES config/workcell_conf.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
