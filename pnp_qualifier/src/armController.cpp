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
void start_competition(ros::NodeHandle & node) 
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) 
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) 
  {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}



// Arm controller class
class armController
{
public:
  armController(ros::NodeHandle & node)
  {
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
    gripperStateSubscriber = node.subscribe("/ariac/gripper/state", 10, &armController::gripperStateCallback, this);
    gripper_client = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
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

  void set_joint_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3) 
  {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    msg.points[0].positions[0] = elbow;
    msg.points[0].positions[1] = linear_pos;
    msg.points[0].positions[2] = shoulder_lift;
    msg.points[0].positions[3] = shoulder_pan;
    msg.points[0].positions[4] = wrist1;
    msg.points[0].positions[5] = wrist2;
    msg.points[0].positions[6] = wrist3;

    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(1.5);
    // ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
  }

  void go_to_home()
  {
    set_joint_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0);
    ROS_INFO("Moved to home");
  }

  void go_to_agv1()
  {
    set_joint_state(1.46, 2.1, -0.54, 1.57, 3.8, -1.57, 1.406);
    ros::Duration(2).sleep();
    ROS_INFO("Moved to AGV1");
  }

  void go_to_agv2()
  {
    set_joint_state(1.46, -2.1, -0.54, 4.71, 3.8, -1.57, 1.406);
    ros::Duration(2).sleep();
    ROS_INFO("Moved to AGV2");
  }

  void travel_left()
  {
    set_joint_state(1.51, 0.0, -1.128, 1.57, 3.77, -1.51, 0.0);
    ROS_INFO("Travelling left");
  }

  void travel_right()
  {
    set_joint_state(1.51, 0.0, -1.128, 4.71, 3.77, -1.51, 0.0);
    ros::Duration(1).sleep();
    ROS_INFO("Travelling right");
  }

  void go_to_belt()
  {
    // set_joint_state(1.65, 0.0, -0.73, 0.0, 3.77, -1.51, 0.0);
    // ros::Duration(2).sleep();
    set_joint_state(1.65, 2.0, -0.73, 0.0, 3.77, -1.51, 0.0);
    ros::Duration(2).sleep();
    ROS_INFO("Moved to belt");
  }

  void grabPart_belt1()
  {
    set_joint_state(1.80, 1.85, -0.71, -0.025, 3.5, -1.57, -0.025);
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

  void grabPart_belt2()
  {
    set_joint_state(1.80, 1.85, -0.712, -0.025, 3.5, -1.57, -0.025);
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

  bool goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group)
  {
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
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

  bool grabPart(geometry_msgs::PoseStamped & pose_1a, moveit::planning_interface::MoveGroupInterface & move_group)
  {
    ros::spinOnce();
    pose_1a.pose.orientation.w = w/mag;
    pose_1a.pose.orientation.x = x/mag;
    pose_1a.pose.orientation.y = y/mag;
    pose_1a.pose.orientation.z = z/mag;

    go_to_home();
    ros::Duration(1).sleep();

    pose_1a.pose.position.z = 0.73;
    this->armController::goToPose(pose_1a.pose, move_group);
    ros::Duration(1).sleep();

    pose_1a.pose.position.z = 0.73;
    this->armController::goToPose(pose_1a.pose, move_group);
    this->armController::grab();
    ros::Duration(2).sleep();
    ros::spinOnce();
    bool success = isGripperAttached();
    ROS_INFO_STREAM(success);
    return success;
  }


  bool depositPart(geometry_msgs::PoseStamped & pose_1b, moveit::planning_interface::MoveGroupInterface & move_group, int agv_pos)
  {
    pose_1b.pose.orientation.w = 0.793;
    pose_1b.pose.orientation.x = 0.0;
    pose_1b.pose.orientation.y = 0.609;
    pose_1b.pose.orientation.z = 0.0;
    ros::spinOnce();

    bool success = false;
    bool gripperState = isGripperAttached();
    while (gripperState == true)
    {
      // go_to_home();
      ros::spinOnce();
      gripperState = isGripperAttached();
      if (gripperState == false)
      {
        break;
      }

      ros::Duration(1).sleep();
      if (agv_pos == 0)
      {
        travel_left();
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }
        ros::Duration(1).sleep();


        go_to_agv1();
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }

        this->armController::goToPose(pose_1b.pose, move_group);
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }


        ros::Duration(2).sleep();
        success = true;
        this->armController::release();
        ros::spinOnce();
      }

      else if (agv_pos == 1)
      {
        travel_right();
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }

        ros::Duration(1).sleep();
        go_to_agv2();
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }


        this->armController::goToPose(pose_1b.pose, move_group);
        ros::spinOnce();
        gripperState = isGripperAttached();
        if (gripperState == false)
        {
          break;
        }

        ros::Duration(2).sleep();
        success = true;
        this->armController::release();
        ros::spinOnce();
      }
    }

    return success;
  }

private:
  //tf::TransformListener listener;
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

  float w = 0.707;
  float x = 0;
  float y = 0.707;
  float z = 0;
  float mag = 1;

  geometry_msgs::PoseStamped pose_1a;
  geometry_msgs::PoseStamped pose_1b;

  pose_1a.pose.orientation.w = w/mag;
  pose_1a.pose.orientation.x = x/mag;
  pose_1a.pose.orientation.y = y/mag;
  pose_1a.pose.orientation.z = z/mag;
  pose_1b.pose.orientation.w = 0.793;
  pose_1b.pose.orientation.x = 0.0;
  pose_1b.pose.orientation.y = 0.609;
  pose_1b.pose.orientation.z = 0.0;

  pose_1a.pose.position.x = 1.20;
  pose_1a.pose.position.y = 2.0;
  pose_1a.pose.position.z = 0.949;

  pose_1b.pose.position.x = 1.2099;
  pose_1b.pose.position.y = 2.0;
  pose_1b.pose.position.z = 0.95;

  bool approach = false;
  bool drop = false;

  

  // Arm.set_joint_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0);
  // ros::Duration(1).sleep();

  Arm.set_joint_state(1.80, 1.85, -0.71, -0.025, 3.5, -1.57, -0.025);
  ros::Duration(5).sleep();

  Arm.goToPose(pose_1a.pose, move_group);



  // osrf_gear::AGVControl srv1;
  // ros::ServiceClient client1 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  // client1.call(srv1);
}