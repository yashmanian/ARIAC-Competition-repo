//
//	Created by Yash Manian on 04/10/2018
//
// Include file

#include<ros/ros.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/Proximity.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <algorithm>
#include <vector>
#include <std_msgs/String.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/JointState.h>
#include <arm_controller/manager.h>

using namespace std;


#ifndef ARM_H
#define ARM_H

class armController
{
public:

  int drop_col = 0;
  float drop_y_pose = 0;
  
	// Default constructors
	armController(ros::NodeHandle & node);

	// Initialize Planner
	moveit::planning_interface::MoveGroupInterface plannerInit();

	// Print joint states
	void printStates();

	// Set Joint State values
	void set_joint_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3, float time);


/*------------------ Motion methods ------------------*/
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

	// Go to bin pickup plan location
	void go_to_bin(int bin_count);

	// Drop Part
	void dropPose(float pos_x, float pos_y, int agv_count);
	
	// Go to faulty part pose
	void FaultyPartPose(string part_type, int agv_count);

	// Grab Faulty Part
	bool grabFaultyPart(string part_type, int agv_count);

	// Grab part from belt
	bool grabBeltPart(int beam_count, string part_type, float velocity);

	// Deposit part on AGV
	bool depositPart(int agv_count, geometry_msgs::Pose finalPose, moveit::planning_interface::MoveGroupInterface & move_group, string part_type);

	// Grab part from bin
	bool grabBinPart(int bin_count, string part_type);
/*------------------ Callback methods ------------------*/
	// Gripper state callback
	void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);

	// Break beam callbacks
	void beam_1_Callback(const osrf_gear::Proximity::ConstPtr &msg);
	void beam_2_Callback(const osrf_gear::Proximity::ConstPtr &msg);
	void beam_3_Callback(const osrf_gear::Proximity::ConstPtr &msg);
	void beam_4_Callback(const osrf_gear::Proximity::ConstPtr &msg);
	void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);


/*------------------ Vacuum Gripper methods ------------------*/
	// Get gripper state
	osrf_gear::VacuumGripperState getGripperState();

	// Check if gripper is attached
	bool isGripperAttached();

	// Grab
	void grab();

	// Release
	void release();


/*------------------ Moveit! methods ------------------*/
		// Plan to pose
	bool goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group);

private:
  bool success = false;
  ros::Publisher joint_trajectory_publisher_;
  ros::Subscriber gripperStateSubscriber;
	ros::Subscriber beam_1_StateSubscriber;
	ros::Subscriber beam_2_StateSubscriber;
	ros::Subscriber beam_3_StateSubscriber;
	ros::Subscriber beam_4_StateSubscriber;
	ros::Subscriber joint_state_subscriber;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool attached = false;
  ros::ServiceClient gripper_client;
  osrf_gear::VacuumGripperState currentGripperState;
  osrf_gear::VacuumGripperControl attach_;
  osrf_gear::VacuumGripperControl detach_;
  tf2_ros::Buffer tfBuffer;
  geometry_msgs::TransformStamped transformStamped;
  sensor_msgs::JointState current_joint_states_;

  bool beamBreak1 = false;
	bool beamBreak2 = false;
	bool beamBreak3 = false;
	bool beamBreak4 = false;

	float curr_elbow = 2.83;
	float curr_y_pos = 0.0;
	float curr_sh_lift = -1.72;
	float curr_sh_pan = 3.31;
	float curr_wrist_1 = 3.60;
	float curr_wrist_2 = -1.51;
	float curr_wrist_3 = 0.14;

	float pick_elbow[4] = {1.917, 1.708, 1.481, 1.645};
	float pick_sh_lift[4] = {-0.455, -0.419, -0.358, -0.405};
	float pick_sh_pan[4] = {3.923, 3.842, 3.765, 3.377};
	float pick_w1[4] = {3.201, 3.389, 3.566, 3.466};
	float pick_w2[4] = {-1.573, -1.572, -1.570, -1.570};
	float pick_w3[4] = {-2.370, -2.443, -2.522, -2.908};

	float trans_elbow[4] = {1.923, 1.702, 1.473, 1.722};
	float trans_sh_lift[4] = {-0.635, -0.572, -0.491, -0.577};
	float trans_sh_pan[4] = {3.967, 3.884, 3.804, 3.386};
	float trans_w1[4] = {3.430, 3.587, 3.736, 3.572};
	float trans_w2[4] = {-1.569, -1.569, -1.569, -1.570};
	float trans_w3[4] = {-2.314, -2.398, -2.477, -2.897};

	float y_offset[4] = {0.618, 0.707, 0.673, 0.354};

	float pick_elbow_pulley[2] = {1.831, 1.318};
	float pick_sh_lift_pulley[2] = {-0.554, -0.388};
	float pick_sh_pan_pulley[2] = {3.927, 3.745};
	float pick_w1_pulley[2] = {3.395, 3.786};
	float pick_w2_pulley[2] = {-1.572, -1.570};
	float pick_w3_pulley[2] = {-2.350, -2.536};

	float trans_elbow_pulley[2] = {1.885, 1.310};
	float trans_sh_lift_pulley[2] = {-0.699, -0.483};
	float trans_sh_pan_pulley[2] = {3.925, 3.769};
	float trans_w1_pulley[2] = {3.536, 3.890};
	float trans_w2_pulley[2] = {-1.569, -1.569};
	float trans_w3_pulley[2] = {-2.357, -2.513};

	float y_offset_pulley[2] = {0.630, 0.667};
	
  float w = 0.707;
  float x = 0;
  float y = 0.707;
  float z = 0;
  float mag = 1;

  // Pan angles for drop and pick up poses
  float delta_pan[5] = {-0.26, -0.219, -0.137, -0.053, -0.011};

  // Drop pose joint angles
	float dep_elbow[5] = {0.836, 0.857, 0.874, 0.857, 0.836};
	float dep_sh_lift[5] = {-0.414, -0.425, -0.433, -0.425, -0.415};
	float dep_w1[5] = {4.294, 4.284, 4.275, 4.284, 4.295};
	float dep_w2[5] = {-1.570, -1.571, -1.571, -1.571, -1.571};
	float dep_w3[5] = {1.310, 1.351, 1.433, 1.517, 1.559};

	// Faulty Part pick up joint angles
	float faulty_elbow[5] = {0.582, 0.743, 0.753, 0.741, 0.707};
	float faulty_sh_lift[5] = {-0.051, -0.127, -0.126, -0.125, -0.112};
	float faulty_w1[5] = {4.151, 4.103, 4.057, 4.095, 4.126};
	float faulty_w2[5] = {-1.568, -1.570, -1.558, -1.569, -1.571};
	float faulty_w3[5] = {1.315, 1.350, 1.435, 1.515, 1.559};

	float faulty_elbow_pulley[5] = {0.765, 0.786, 0.808, 0.793, 0.769};
	float faulty_sh_lift_pulley[5] = {-0.195, -0.205, -0.214, -0.208, -0.196};
	float faulty_w1_pulley[5] = {4.154, 4.144, 4.113, 4.118, 4.128};
	float faulty_w2_pulley[5] = {-1.569, -1.571, -1.565, -1.571, -1.575};
	float faulty_w3_pulley[5] = {-4.972, 1.351, 1.434, 1.517, 1.560};

	float def_vel = 0.2;
};

#endif