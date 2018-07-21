
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
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


// Global flags
bool qa1_state;
bool qa2_state;
bool READY_flag = false;


// Ready state global variables
string bin_type;
int bin_number;
string part_type;
int Agv_Number = 0;
float Velocity;
bool agv1_fail = false;
bool agv2_fail = false;
bool faulty_success;

float w = 0.707;
float x = 0;
float y = 0.707;
float z = 0;
float mag = 1;

// Planning
geometry_msgs::Pose initialPose;
geometry_msgs::Pose finalPose;
geometry_msgs::Pose faultyPose1;
geometry_msgs::Pose faultyPose2;
geometry_msgs::PoseStamped faultyPose;
string faultyPart1;
string faultyPart2;



// Faulty parts
// string qa_id[2]={"quality_control_sensor_1_frame","quality_control_sensor_2_frame"}; 
// tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);
// geometry_msgs::TransformStamped qa_to_world_frame;


/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) 
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
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
  } 
  else 
  {
    ROS_INFO("Competition started!");
  }
}

void qa1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) 
{
	osrf_gear::LogicalCameraImage scan=*image_msg;
	osrf_gear::LogicalCameraImage LogicalImage_1=scan;
	
	if(!LogicalImage_1.models.empty()) 
	{
		qa1_state = true;
		faultyPose1 = LogicalImage_1.models[0].pose;
		faultyPart1 = LogicalImage_1.models[0].type;
		// ROS_INFO("No faulty parts!");
	}
	else
	{
		qa1_state = false;
		// ROS_INFO("Faulty parts found!");
	}
}

void qa2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) 
{
	osrf_gear::LogicalCameraImage scan=*image_msg;
	osrf_gear::LogicalCameraImage LogicalImage_2=scan;

	if(!LogicalImage_2.models.empty()) 
	{
		qa2_state = true;
		faultyPose2 = LogicalImage_2.models[0].pose;
		faultyPart2 = LogicalImage_2.models[0].type;
		// ROS_INFO("No faulty parts!");
	}
	else
	{
		qa2_state = false;
		// qa_part2 = LogicalImage_2.models[0].pose;
		// ROS_INFO("Faulty parts found!");
	}
}


void ReadyState_callback(const arm_controller::manager::ConstPtr & msg)
{
	arm_controller::manager data = *msg;
	ROS_INFO_STREAM("Current Order: " << data);
	geometry_msgs::PoseStamped temp_init = data.Initial_Pose;
	geometry_msgs::PoseStamped temp_final = data.Final_Pose;

	bin_type = data.bin_type;
	bin_number = data.bin_number;
	part_type = data.part_type;
	initialPose.position = temp_init.pose.position;
	finalPose.position = temp_final.pose.position;
	initialPose.orientation.w = w/mag;
	initialPose.orientation.x = x/mag;
	initialPose.orientation.y = y/mag;
	initialPose.orientation.z = z/mag;
	finalPose.orientation.w = w/mag;
	finalPose.orientation.x = x/mag;
	finalPose.orientation.y = y/mag;
	finalPose.orientation.z = z/mag;
	Agv_Number = data.Agv_Number;
	Velocity = data.Velocity;
	READY_flag = true;
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "qual3_node");
	int loop = 0;
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(1);
  spinner.start();

  	// Setup Variables
	bool belt;
	bool bin;
	bool grab_success;
	bool drop_success;

  	// States
	int READY = 1;
	int GET_OBJ = 2;
	int DROP_OBJ = 3;
	int DONE = 4;
	int ERROR = 5;
	int FAULTY = 6;
	int EXIT = 7;

  int CURR_STATE = 0;

  int prev_agv_number = Agv_Number;
  // Publisher data
  ros::Publisher StatePub = node.advertise<std_msgs::Bool>("/armControllerFlag",1000);
	std_msgs::Bool Flag;

	// Quality control 
	ros::Subscriber qa1_subscriber = node.subscribe("/ariac/quality_control_sensor_1", 10, qa1_callback);
	ros::Subscriber qa2_subscriber = node.subscribe("/ariac/quality_control_sensor_2", 10, qa2_callback);

	// Get Pose
	ros::Subscriber ReadyState_subscriber = node.subscribe("ReadyState", 30, ReadyState_callback);

	// start_competition(node);

  	// Initialize Arm planner
	armController Arm(node);
	moveit::planning_interface::MoveGroupInterface move_group = Arm.plannerInit();

	Arm.go_to_home();

	CURR_STATE = READY;

/*------------------READY STATE------------------*/

	while (CURR_STATE != EXIT)
	{
		
		// Ready State operations
		if (CURR_STATE == READY)
		{	
			prev_agv_number = Agv_Number;
			while (READY_flag == false)
			{
				// ROS_INFO("Spinning-----------");
				ros::spinOnce();
			}

			if (bin_type == "bin")
			{
				bin = true;
				belt = false;
			}
			else if(bin_type == "belt")
			{
				bin = false;
				belt = true;
			}
			
			READY_flag = false;
			if (prev_agv_number != Agv_Number)
			{
				agv1_fail = false;
				agv2_fail = false;
			}
			CURR_STATE = GET_OBJ;
			Arm.release();
			ROS_INFO_STREAM("Going to GET_OBJ state" << CURR_STATE);
		}



/*------------------GET_OBJ STATE------------------*/
		if (CURR_STATE == GET_OBJ)
		{
			if (belt == true)
			{
				grab_success = Arm.grabBeltPart(bin_number, part_type, Velocity);
				if (grab_success == true)
				{
					ROS_INFO("Got Part!");
				}
				else if (grab_success == false)
				{
					ROS_INFO("Didn't get Part!");
				}
			}

			else if (bin == true)
			{
				grab_success = Arm.grabBinPart(bin_number, part_type);
				if (grab_success == true)
				{
					ROS_INFO("Got Part!");
				}
				else if (grab_success == false)
				{
					ROS_INFO("Didn't get Part!");
				}
			}

			if (grab_success == true)
			{
				CURR_STATE = DROP_OBJ;
				ROS_INFO("Going to DROP_OBJ state");
			}
			else
			{
				CURR_STATE = ERROR;
				ROS_INFO("Staying in GET_OBJ state");
			}
		}


/*------------------DROP_OBJ STATE------------------*/
		if (CURR_STATE == DROP_OBJ)
		{
			drop_success = Arm.depositPart(Agv_Number, finalPose, move_group, part_type);

			if (drop_success == true)
			{
				CURR_STATE = DONE;
				ROS_INFO("Part successfully deposited!");
				ROS_INFO("Going to DONE state");
			}
			else
			{
				CURR_STATE = ERROR;
				ROS_INFO("Part dropped during transit!");
				ROS_INFO("Going to ERROR state");	
			}
		}

/*------------------ERROR STATE------------------*/
		if (CURR_STATE == ERROR)
		{
			Arm.release();
			Flag.data = false;
			StatePub.publish(Flag);
			ROS_INFO("Published Flag False!");
			CURR_STATE = READY;
			ROS_INFO("Going to READY state");	
		}



/*------------------DONE STATE------------------*/
		if (CURR_STATE == DONE)
		{
			bool qa_flag;
			ros::spinOnce();
			if (Agv_Number == 1)
			{
				qa_flag = qa1_state;
				faultyPose.pose.position.x = faultyPose1.position.x;
				faultyPose.pose.position.y = faultyPose1.position.y;
				faultyPose.pose.position.z = faultyPose1.position.z;
				faultyPose.pose.orientation.x = x/mag;
				faultyPose.pose.orientation.y = y/mag;
				faultyPose.pose.orientation.z = z/mag;
				faultyPose.pose.orientation.w = w/mag;
			}
			else if(Agv_Number == 2)
			{
				qa_flag = qa2_state;
				faultyPose.pose.position.x = faultyPose2.position.x;
				faultyPose.pose.position.y = faultyPose2.position.y;
				faultyPose.pose.position.z = faultyPose2.position.z;
				faultyPose.pose.orientation.x = x/mag;
				faultyPose.pose.orientation.y = y/mag;
				faultyPose.pose.orientation.z = z/mag;
				faultyPose.pose.orientation.w = w/mag;
			}

			if (qa_flag == true)
			{
				ROS_INFO("Faulty Part found");

				// qa_to_world_frame = tfBuffer.lookupTransform( "world", qa_id[Agv_Number - 1], ros::Time(0), ros::Duration(1.0));
				// ROS_INFO_STREAM(qa_id[Agv_Number - 1]);

	  			// tf2::doTransform(faultyPose, faultyPose, qa_to_world_frame);
	  			// ROS_INFO_STREAM("Faulty Pose -------------------" << faultyPose);

				ROS_INFO_STREAM("Curr Col: " << Arm.drop_col << " Curr_y_pose: " << Arm.drop_y_pose);
				/* Temporary measures*/

				/* To be replaced*/

				CURR_STATE = FAULTY;
				ROS_INFO("Going to Faulty state");
			}
			else
			{
				ROS_INFO("No Faulty Parts found");
				Flag.data = true;
				StatePub.publish(Flag);
				ROS_INFO("Published Flag True!");

				CURR_STATE = READY;
				ROS_INFO("Going to READY state");
			}

		}

/*------------------FAULTY STATE------------------*/
		if (CURR_STATE == FAULTY)
		{
			ROS_INFO("Entered Faulty State----------------");
			if (Agv_Number == 1 && agv1_fail == false)
			{
				faulty_success = Arm.grabFaultyPart(part_type, Agv_Number);
				ros::Duration(1).sleep();
			}
			else if(Agv_Number == 2 && agv2_fail == false)
			{
				faulty_success = Arm.grabFaultyPart(part_type, Agv_Number);
				ros::Duration(1).sleep();			
			}

			if (faulty_success == false)
			{
				if (Agv_Number == 1)
				{
					agv1_fail = true;
				}
				else if(Agv_Number == 2)
				{
					agv2_fail = true;
				}
			}

			if (Agv_Number == 1 && agv1_fail == false)
			{
				Flag.data = false;	// Change to false later
				StatePub.publish(Flag);
				ROS_INFO_STREAM("Published Flag False!" << Flag);
				ROS_INFO("Going to READY state");
			}
			else if (Agv_Number == 2 && agv2_fail == false)
			{
				Flag.data = false;	// Change to false later
				StatePub.publish(Flag);
				ROS_INFO_STREAM("Published Flag False!" << Flag);
				ROS_INFO("Going to READY state");
			}
			else
			{
				Flag.data = true;	// Change to false later
				StatePub.publish(Flag);
				ROS_INFO_STREAM("Published Flag True!" << Flag);
				ROS_INFO("Going to READY state");
			}
			CURR_STATE = READY;
		}

/*------------------EXIT STATE------------------*/
		if (CURR_STATE == EXIT)
		{
			ROS_INFO("Exiting State Machine!");
			break;
		}

	}
}

