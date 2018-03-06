#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <tf2_ros/transform_listener.h>
#include<osrf_gear/AGVControl.h>


/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}


class armController
{
public:
  armController(ros::NodeHandle & node)
  {
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
    gripperStateSubscriber = node.subscribe("/ariac/gripper/state", 10, &armController::gripperStateCallback, this);
    gripper_client = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    
    home_pose.position.x = -0.1;
    home_pose.position.y = 0.79;
    home_pose.position.z = 0.9;
    home_pose.orientation.w = w/mag;
    home_pose.orientation.x = x/mag;
    home_pose.orientation.y = y/mag;
    home_pose.orientation.z = z/mag;

    mag = sqrt(x*x + y*y + z*z + w*w);

    mid1_pose.position.x = -0.0426;
    mid1_pose.position.y = 1.6;
    mid1_pose.position.z = 0.9;
    mid1_pose.orientation.w = w/mag;
    mid1_pose.orientation.x = x/mag;
    mid1_pose.orientation.y = y/mag;
    mid1_pose.orientation.z = z/mag;

    mid2_pose.position.x = -0.0426;
    mid2_pose.position.y = 3.1;
    mid2_pose.position.z = 0.9;
    mid2_pose.orientation.w = w/mag;
    mid2_pose.orientation.x = x/mag;
    mid2_pose.orientation.y = y/mag;
    mid2_pose.orientation.z = z/mag;


    agv_pose.position.x = 0.3;
    agv_pose.position.y = 3.1;
    agv_pose.position.z = 0.9;
    agv_pose.orientation.w = w/mag;
    agv_pose.orientation.x = x/mag;
    agv_pose.orientation.y = y/mag;
    agv_pose.orientation.z = z/mag;
  }

  moveit::planning_interface::MoveGroupInterface plannerInit()
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

  bool goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group)
  {
    move_group.setPoseTarget(target_pose);
    move_group.move();
    move_group.plan(my_plan);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
  }

  void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) 
  {
    currentGripperState = *msg;
    attached = msg->attached;
  }


  osrf_gear::VacuumGripperState getGripperState() 
  {
    ros::spinOnce();
    return currentGripperState;
  }

  bool isGripperAttached() 
  {
    ros::spinOnce();
    return attached;
  }

  void grab() 
  {
    ROS_INFO("Enable gripper");
    attach_.request.enable = 1;
    gripper_client.call(attach_);
    ros::spinOnce();
    ROS_INFO("I %s got the part", isGripperAttached()? "have": "haven't");
  }

  void release() 
  {
    ROS_INFO("Release gripper");
    attach_.request.enable = 0;
    gripper_client.call(attach_);
    ros::spinOnce();
  }

  void dropPart(geometry_msgs::Pose & pose_1a, geometry_msgs::Pose & pose_1b, moveit::planning_interface::MoveGroupInterface & move_group)
  {
    pose_1a.orientation.w = w/mag;
    pose_1a.orientation.x = x/mag;
    pose_1a.orientation.y = y/mag;
    pose_1a.orientation.z = z/mag;
    pose_1b.orientation.w = w/mag;
    pose_1b.orientation.x = x/mag;
    pose_1b.orientation.y = y/mag;
    pose_1b.orientation.z = z/mag;

    this->armController::goToPose(pose_1a, move_group);
    ros::Duration(3).sleep();
    this->armController::goToPose(pose_1b, move_group);
    this->armController::grab();
    ros::Duration(3).sleep();
    this->armController::goToPose(pose_1a, move_group);
    ros::Duration(3).sleep();
    this->armController::goToPose(mid1_pose, move_group);
    ros::Duration(3).sleep();
    this->armController::goToPose(mid2_pose, move_group);
    ros::Duration(3).sleep();
    this->armController::goToPose(agv_pose, move_group);
    this->armController::release();
    ros::Duration(3).sleep();
    this->armController::goToPose(mid1_pose, move_group);
    ros::Duration(3).sleep();
    this->armController::goToPose(mid2_pose, move_group);
    ros::Duration(3).sleep();
  }

private:
  tf::TransformListener listener;
  bool success = false;
  ros::Publisher joint_trajectory_publisher_;
  ros::Subscriber gripperStateSubscriber;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  float arrivalTime = 0.5;
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

  geometry_msgs::Pose home_pose;
  geometry_msgs::Pose mid1_pose;
  geometry_msgs::Pose mid2_pose;
  geometry_msgs::Pose agv_pose;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ariac_example_node");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  start_competition(node);

  armController Arm(node);
  moveit::planning_interface::MoveGroupInterface move_group = Arm.plannerInit();
  bool success = false;

  geometry_msgs::Pose pose_1a;
  geometry_msgs::Pose pose_1b;
  geometry_msgs::Pose pose_2a;
  geometry_msgs::Pose pose_2b;
  geometry_msgs::Pose pose_3a;
  geometry_msgs::Pose pose_3b;
  geometry_msgs::Pose pose_4a;
  geometry_msgs::Pose pose_4b;
  geometry_msgs::Pose pose_5a;
  geometry_msgs::Pose pose_5b;

  pose_1a.position.x = -0.1;
  pose_1a.position.y = -0.35;
  pose_1a.position.z = 0.9;

  pose_1b.position.x = -0.1;
  pose_1b.position.y = -0.35;
  pose_1b.position.z = 0.73;

  pose_2a.position.x = -0.1;
  pose_2a.position.y = -0.47;
  pose_2a.position.z = 0.9;

  pose_2b.position.x = -0.1;
  pose_2b.position.y = -0.47;
  pose_2b.position.z = 0.73;

  pose_3a.position.x = -0.1;
  pose_3a.position.y = -0.6;
  pose_3a.position.z = 0.9;

  pose_3b.position.x = -0.1;
  pose_3b.position.y = -0.6;
  pose_3b.position.z = 0.73;

  pose_4a.position.x = -0.1;
  pose_4a.position.y = 0.43;
  pose_4a.position.z = 0.9;

  pose_4b.position.x = -0.1;
  pose_4b.position.y = 0.43;
  pose_4b.position.z = 0.723;

  pose_5a.position.x =-0.1;
  pose_5a.position.y = 0.23;
  pose_5a.position.z = 0.9;

  pose_5b.position.x = -0.1;
  pose_5b.position.y = 0.23;
  pose_5b.position.z = 0.726;


// Motion
  Arm.dropPart(pose_1a, pose_1b, move_group);
  Arm.dropPart(pose_2a, pose_2b, move_group);
  Arm.dropPart(pose_3a, pose_3b, move_group);
  Arm.dropPart(pose_4a, pose_4b, move_group);
  Arm.dropPart(pose_5a, pose_5b, move_group);

  osrf_gear::AGVControl srv1;
  ros::ServiceClient client1 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  client1.call(srv1);
}