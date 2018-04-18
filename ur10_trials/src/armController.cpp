//
//	Created by Yash Manian on 04/10/2018
//

#include "../include/armController.h" 

// Constructor class
armController::armController(ros::NodeHandle & node)
{
  joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
  gripperStateSubscriber = node.subscribe("/ariac/gripper/state", 10, &armController::gripperStateCallback, this);
  gripper_client = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
}


// Initialize planner
moveit::planning_interface::MoveGroupInterface armController::plannerInit()
{
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlannerId("RRTkConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  return move_group;
}

// Set joint states
void armController::set_joint_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3, float time) 
{
  trajectory_msgs::JointTrajectory msg;
  msg.joint_names.clear();
  msg.joint_names.push_back("elbow_joint");
  msg.joint_names.push_back("linear_arm_actuator_joint");
  msg.joint_names.push_back("shoulder_lift_joint");
  msg.joint_names.push_back("shoulder_pan_joint");
  msg.joint_names.push_back("wrist_1_joint");
  msg.joint_names.push_back("wrist_2_joint");
  msg.joint_names.push_back("wrist_3_joint");

  msg.points.resize(1);

  msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
  msg.points[0].positions[0] = elbow;
  msg.points[0].positions[1] = linear_pos;
  msg.points[0].positions[2] = shoulder_lift;
  msg.points[0].positions[3] = shoulder_pan;
  msg.points[0].positions[4] = wrist1;
  msg.points[0].positions[5] = wrist2;
  msg.points[0].positions[6] = wrist3;

  msg.points[0].time_from_start = ros::Duration(time);
  joint_trajectory_publisher_.publish(msg);
}


// Predefined joint states and locations to speed up the process
void armController::go_to_home()
{
  set_joint_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0, 1.0);
  ROS_INFO("Moved to home");
}

void armController::go_to_agv1()
{
  set_joint_state(1.46, 2.1, -0.54, 1.57, 3.8, -1.57, 1.406, 1.5);
  ros::Duration(2).sleep();
  ROS_INFO("Moved to AGV1");
}

void armController::go_to_agv2()
{
  set_joint_state(1.46, -2.1, -0.54, 4.71, 3.8, -1.57, 1.406, 1.5);
  ros::Duration(2).sleep();
  ROS_INFO("Moved to AGV2");
}

void armController::travel_left()
{
  set_joint_state(1.51, 0.0, -1.128, 1.57, 3.77, -1.51, 0.0, 1.5);
  ROS_INFO("Travelling left");
}

void armController::travel_right()
{
  set_joint_state(1.51, 0.0, -1.128, 4.71, 3.77, -1.51, 0.0, 1.5);
  ros::Duration(1).sleep();
  ROS_INFO("Travelling right");
}

void armController::go_to_belt()
{
  set_joint_state(1.65, 2.0, -0.73, 0.0, 3.77, -1.51, 0.0, 1.5);
  ros::Duration(2).sleep();
  ROS_INFO("Moved to belt");
}

void armController::grabPart_belt1()
{
  set_joint_state(1.67, 2.0, -0.69, 0.049, 3.73, -1.57, 0.049, 1.5);
  ros::Duration(2).sleep();
  
  while(1)
  {
    ros::spinOnce();
    this->armController::grab();
    bool gripperState = isGripperAttached();
    if (gripperState == true)
    {
      break;
    }
  }
  ROS_INFO("Moved to belt");
}

void armController::grabPart_belt2()
{
  set_joint_state(1.80, 1.85, -0.712, -0.025, 3.5, -1.57, -0.025, 1.5);
  ros::Duration(2).sleep();
  
  while(1)
  {
    ros::spinOnce();
    this->armController::grab();
    bool gripperState = isGripperAttached();
    if (gripperState == true)
    {
      break;
    }
  }
  ROS_INFO("Moved to belt");
}


// Gripper state callbacks and actuation functions
void armController::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) 
{
  currentGripperState = *msg;
  attached = msg->attached;
}


osrf_gear::VacuumGripperState armController::getGripperState() 
{
  ros::spinOnce();
  return currentGripperState;
}

bool armController::isGripperAttached() 
{
  ros::spinOnce();
  return attached;
}

void armController::grab() 
{
  ROS_INFO("Enable gripper");
  attach_.request.enable = 1;
  gripper_client.call(attach_);
  ros::spinOnce();
  ROS_INFO("I %s got the part", isGripperAttached()? "have": "haven't");
}

void armController::release() 
{
  ROS_INFO("Release gripper");
  attach_.request.enable = 0;
  gripper_client.call(attach_);
  ros::spinOnce();
}


// Plan to pose using MoveIt!
bool armController::goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group)
{
  move_group.setPoseTarget(target_pose);
  move_group.plan(my_plan);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  return success;
}
