/**
* @author Vaibhav Bhilare
* @copyright 2018, Vaibhav Bhilare
*/

#include<ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
// #include <osrf::trajectory_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <algorithm>
#include <vector>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <competition/manager.h>


#ifndef COMPETITION_INCLUDE_MANAGER_H_
#define COMPETITION_INCLUDE_MANAGER_H_

class manager {
public:
	explicit manager(ros::NodeHandle & node);
	int argc;
	char ** argv;
	std::string competition_state_;
	double current_score_;
	bool has_been_zeroed_;
	std::vector<osrf_gear::Kit> kitList;
	osrf_gear::LogicalCameraImage LogicalImage_1;
	std::vector<osrf_gear::Order> received_order;
	std::vector<osrf_gear::KitObject> backlogKit;
	int currentAgv;
	std::vector<std::string> agvs{"agv1_load_point_frame","agv2_load_point_frame"};
	std::vector<osrf_gear::KitObject> currentObjectList;
	double Velocity=0;
	double Y_Previous;
	double Y_Now;
	double Time_Previous;
	double Time_Now;
	double Dead_Time=0;
	bool Same_Object=false;
	int armFeedback=2;
	ros::Subscriber current_score_subscriber;
	ros::Subscriber competition_state_subscriber;
	ros::Subscriber orders_subscriber;
	ros::Subscriber logical_camera_1_subscriber;
	ros::Subscriber break_beam_subscriber;
	ros::Subscriber armController;
	tf2_ros::Buffer tfBuffer;
	ros::ServiceClient material_client,agv1_client,agv2_client;
	ros::Publisher ReadyState;
	int binNumber;
	std::string binType, partType;
	std::vector<osrf_gear::Model> Conveyor_Objects;
	std::vector<double> Conveyor_Times;
	std::vector<double> Conveyor_Y;
	geometry_msgs::PoseStamped initialPose, finalPose;
	std::string Logical_Camera_Frame;
	std::string currentKitId,backlogKitId; 
	
	void current_score_callback(const std_msgs::Float32::ConstPtr & msg);
	void competition_state_callback(const std_msgs::String::ConstPtr & msg);
	void order_callback(const osrf_gear::Order::ConstPtr & currentOrder);
	void logical_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void break_beam_1_callback(const osrf_gear::Proximity::ConstPtr & msg);
	void logical_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void logical_camera_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void logical_camera_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void logical_camera_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void armController_Callback(const std_msgs::Bool::ConstPtr& CompletionFlag);

	void competition_manager(ros::NodeHandle & node);
	void start_competition(ros::NodeHandle & node);
	void order_manager(std::vector<osrf_gear::Order>& received_order);
	std::vector<osrf_gear::KitObject> kit_manager(osrf_gear::Kit currentKit);
	void agv_switch();
	void switch_kit();
	virtual ~manager();
};
#endif  // COMPETITION_INCLUDE_MANAGER_H_