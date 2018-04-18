
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/AGVControl.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <vector>

#include "../include/armController.h" 


// Global Variables
std::vector<osrf_gear::Kit> kitList;
std::vector<osrf_gear::Model> invent_1,invent_2;
osrf_gear::LogicalCameraImage LogicalImage_1,LogicalImage_2;
std::string logical_camera_frame;
std::string part_frame;
std::string part_frame_no;
int piston_id=1;
int gear_id=1;
int piston_orders=0;
int gear_orders=0;
int piston_numbers;
int gear_numbers;
bool beamBreak = false;
ros::Time time_stamp;
double time_sec;

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


void beamCallback(const osrf_gear::Proximity::ConstPtr &msg)
{
	osrf_gear::Proximity beamState = *msg;
	beamBreak = msg->object_detected;
	time_stamp = msg->header.stamp;
	time_sec = ros::Time::now().toSec();
	ROS_INFO_STREAM(beamBreak);
	ROS_INFO_STREAM(time_stamp);
	ROS_INFO_STREAM(time_sec);
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) 
{
	// size_t number_of_valid_ranges = std::count_if(msg->ranges.begin(), msg->ranges.end(), std::isfinite<float>);
 //  	if (number_of_valid_ranges > 0) 
 //  	{
 //    	ROS_INFO_STREAM(number_of_valid_ranges);
	// }
}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "qual1_node");
	int loop = 0;
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(1);
  	spinner.start();

	ROS_INFO("Setup complete.");
	start_competition(node);
	ros::Duration(0.5).sleep();
	ros::Subscriber beamStateSubscriber = node.subscribe("/ariac/break_beam", 10, beamCallback);
  	ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler", 10, laser_profiler_callback);

	armController Arm(node);
	moveit::planning_interface::MoveGroupInterface move_group = Arm.plannerInit();


	Arm.set_joint_state(1.65, 0.0, -0.73, 0.0, 3.77, -1.51, 0.0, 1);
	while (loop == 0)
	{
		if (beamBreak == true)
		{
			Arm.set_joint_state(1.67, 0.0, -0.65, 0.0, 3.73, -1.57, 0.049, 0.9);
			while (beamBreak == true)
			{
				ros::spinOnce();
				Arm.grab();
				bool gripperState = Arm.isGripperAttached();
				if (gripperState == true)
				{
					break;
				}
			}
			Arm.set_joint_state(1.65, 0.0, -0.73, 0.0, 3.77, -1.51, 0.0, 1);
			ros::Duration(1).sleep();

			Arm.go_to_agv1();
			ros::Duration(1).sleep();
			Arm.release();

			ROS_INFO("Grabbed");
			loop = 1;
		}
	}


}

