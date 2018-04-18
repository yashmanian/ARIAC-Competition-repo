//
//	Created by Yash Manian on 04/10/2018
//
// Include file

#include<ros/ros.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <algorithm>
#include <vector>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


#ifndef ARM_H
#define ARM_H

class armController
{
public:
	// Default constructors
	armController(ros::NodeHandle & node);

	// Initialize Planner
	moveit::planning_interface::MoveGroupInterface plannerInit();

	// Set Joint State values
	void set_joint_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3, float time);

	// Go to home position
	void go_to_home();

	// Go to AGV 1
	void go_to_agv1();

	// Go to AGV 2
	void go_to_agv2();

	// Initialize left travel
	void travel_left();

	// Initialize right travel
	void travel_right();

	// Go to belt pickup position
	void go_to_belt();

	// Pick up part from belt, part 1
	void grabPart_belt1();

	// Pick up part from belt, part 2
	void grabPart_belt2();

	// Plan to pose
	bool goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group);

	// Gripper state callback
	void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);

	// Get gripper state
	osrf_gear::VacuumGripperState getGripperState();

	// Check if gripper is attached
	bool isGripperAttached();

	// Grab
	void grab();

	// Release
	void release();

private:
  bool success = false;
  ros::Publisher joint_trajectory_publisher_;
  ros::Subscriber gripperStateSubscriber;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool attached = false;
  ros::ServiceClient gripper_client;
  osrf_gear::VacuumGripperState currentGripperState;
  osrf_gear::VacuumGripperControl attach_;
  osrf_gear::VacuumGripperControl detach_;
  tf2_ros::Buffer tfBuffer;
  geometry_msgs::TransformStamped transformStamped;

  float w = 0.707;
  float x = 0;
  float y = 0.707;
  float z = 0;
  float mag = 1;
};

#endif