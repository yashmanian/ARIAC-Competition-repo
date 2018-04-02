#include<ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
// #include <osrf::trajectory_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
//#include <osrf_gear_srv/GetMaterialLocations.h>
//#include <osrf_gear_srv/AGVControl.h>
//#include <osrf_gear_srv/VacuumGripperControl.h>
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

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

// Global Variables
std::vector<osrf_gear::KitObject> objectList;
std::vector<osrf_gear::Model> invent;
osrf_gear::LogicalCameraImage LogicalImage;
std::string logical_camera_frame;
std::string part_frame;
std::string part_frame_no;
int piston_id=1;
int gear_id=1;
int piston_orders=0;
int gear_orders=0;
int piston_numbers;
int gear_numbers;


void plan(std::string type, int quantity, int available){
ROS_INFO_STREAM(type<<" : "<<quantity<<" : "<<available);
}

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

// Competition Class
class MyCompetitionClass
{
public:
  std::string competition_state_;
  double current_score_;
bool has_been_zeroed_;
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0),has_been_zeroed_(false)
  {
}


// Score Callback
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }


// Order Callback
void order_callback(const osrf_gear::Order::ConstPtr & currentOrder) {
std::vector<osrf_gear::Order> received_order;
received_order.push_back(*currentOrder);
for(osrf_gear::KitObject& object :received_order[0].kits[0].objects){
objectList.push_back(object);
}
}

/// Called when a new LogicalCameraImage message is received.
void logical_camera_1_callback(
const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
osrf_gear::LogicalCameraImage scan=*image_msg;
piston_numbers=image_msg->models.size();
LogicalImage=scan;
while(scan.models.size()==0){
ros::Duration(0.5).sleep();
}
std::vector<osrf_gear::Model> allObjects=scan.models;
for (osrf_gear::Model& object : allObjects){
invent.push_back(object);
}
}

void logical_camera_2_callback(
const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
osrf_gear::LogicalCameraImage scan=*image_msg;
gear_numbers=image_msg->models.size();
LogicalImage=scan;
while(scan.models.size()==0){
ros::Duration(0.5).sleep();
}
std::vector<osrf_gear::Model> allObjects=scan.models;
for (osrf_gear::Model& object : allObjects){
invent.push_back(object);
}
}


//osrf_gear::LogicalCameraImage scanShopFloor(){
//osrf_gear::LogicalCameraImage Image=Logical_Camera_Data();
//return Image;
//}

};

// PDDL function

bool PDDL(string parts, int n_bf_piston, int n_bf_gear, int n_tray_piston, int n_tray_gear)
{

	ofstream curr_file;
	bool success = false;

	int n_curr_tray_piston = 0;
	int n_curr_tray_gear = 0;

	string line;
	string tray = "current-part-in-tray ";
	string bin = "current-quantity-in-bin ";
	string order = "part-order-in-tray ";

	string str_piston = " piston ";
	string str_bin_piston = " binforpiston ";
	string piston = "";
	string bin_piston = "";
	string curr_tray_piston = "";
	string bf_piston = "";
	string tray_piston = "";
	string inbin_piston = "";
	string goal_piston1 = "";
	string goal_piston2 = "";

	string str_gear = " gear ";
	string str_bin_gear = " binforgear ";
	string gear = "";
	string bin_gear = "";
	string curr_tray_gear = "";
	string bf_gear = "";
	string tray_gear = "";
	string inbin_gear = "";
	string goal_gear1 = "";
	string goal_gear2 = "";

	size_t found_piston = parts.find(str_piston);
	size_t found_gear = parts.find(str_gear);

	// Checks for parts in string

	if (found_piston != string::npos)
	{
		piston = str_piston;
		bin_piston = str_bin_piston;
		curr_tray_piston = "    (=(" + tray + piston + ") " + to_string(n_curr_tray_piston) + ")\n";
		bf_piston = "    (=(" + bin + str_bin_piston + ") " + to_string(n_bf_piston) + ")\n";
		tray_piston = "    (=(" + order + piston + ") " + to_string(n_tray_piston) + ")\n";
		inbin_piston = "    (inbin " + piston + str_bin_piston  + ")\n";
		goal_piston1 = "      (= (" + tray + piston + ") (" + order + piston + "))\n";
		goal_piston2 = "      (>= (" + bin + str_bin_piston + ") 0)\n";
	}
	if (found_gear != string::npos)
	{
		gear = str_gear;
		bin_gear = str_bin_gear;
		curr_tray_gear = "    (=(" + tray + gear + ") " + to_string(n_curr_tray_gear) + ")\n";
		bf_gear = "    (=(" + bin + str_bin_gear + ") " + to_string(n_bf_gear) + ")\n";
		tray_gear = "    (=(" + order + gear + ") " + to_string(n_tray_gear) + ")\n";
		inbin_gear = "    (inbin " + gear + str_bin_gear  + ")\n";
		goal_gear1 = "      (= (" + tray + gear + ") (" + order + gear + "))\n";
		goal_gear2 = "      (>= (" + bin + str_bin_gear + ") 0)\n";
	}

	curr_file.open("temp.txt", ios::out | ios::binary);

	curr_file << "(define (problem qual1problem)\n";
	curr_file << "  (:domain pnpdomain)\n";
	curr_file << "  ;;objects\n";
	curr_file << "  (:objects \n";
	curr_file << "    r - robot\n";
	curr_file << "    tray - tray\n";
	curr_file << "    " + piston + gear + "- parttype\n";
	curr_file << "    " + bin_piston + bin_gear + "- bin\n";
	curr_file << "  )\n";
	curr_file << "\n";
	curr_file << "  ;;initial state\n";
	curr_file << "  (:init \n";
	curr_file << curr_tray_piston;
	curr_file << curr_tray_gear;
	curr_file << bf_piston;
	curr_file << bf_gear;
	curr_file << tray_piston;
	curr_file << tray_gear;
	curr_file << inbin_piston;
	curr_file << inbin_gear;
	curr_file << "    (handempty r)\n";
	curr_file << "    (athome r)\n";
	curr_file << "    )\n";
	curr_file << "\n";
	curr_file << "  (:goal (and \n";
	curr_file << goal_piston1;
	curr_file << goal_gear1;
	curr_file << goal_piston2;
	curr_file << goal_gear2;
	curr_file << "      )\n";
	curr_file << "    )\n";
	curr_file << "  )\n";


	curr_file.close();
	remove("qual1-problem.pddl");
	rename("temp.txt", "qual1-problem.pddl");

	success = true;
	return success;
}



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "qual1_node");

  ros::NodeHandle node;
	ros::Publisher partLocation = node.advertise<geometry_msgs::Pose>("partLocation", 10);


  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);


  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_1_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 1,
    &MyCompetitionClass::logical_camera_1_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_2' topic.
  ros::Subscriber logical_camera_2_subscriber = node.subscribe(
    "/ariac/logical_camera_2", 1,
    &MyCompetitionClass::logical_camera_2_callback, &comp_class);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::Rate rate(10.0);

  ROS_INFO("Setup complete.");
  start_competition(node);
 	ros::Duration(0.5).sleep();
	ros::spinOnce();  // This executes callbacks on new data until ctrl-c.


// Higher Level Planning

	for(osrf_gear::KitObject object_count: objectList)
	{
		if(object_count.type=="piston_rod_part")
		{
			piston_orders++;
		}
		else if(object_count.type=="gear_part")
		{
			gear_orders++;
		}
	}


	std::vector<std::string>order_sorted={"piston", "gear"};
	std::vector<int>order_quantities={piston_orders,gear_orders};
	std::vector<int>bin_parts={piston_numbers,gear_numbers};
	


	string parts = "";
	for (int i = 0; i < order_sorted.size(); i++)
	{
		parts = parts + " " + order_sorted[i] + " ";
	}

	bool PDDL_pointer = PDDL(parts, bin_parts[0], bin_parts[1], order_quantities[0], order_quantities[1]);
	cout << " Parts String : " << parts << '\n';
	cout << " Bin parts : " << bin_parts[0] << "," << bin_parts[1] << '\n';
	cout << " Order parts :" << order_quantities[0] << "," << order_quantities[1] << '\n';
	cout << " Success : " << PDDL_pointer << '\n'; 

	order_sorted.erase(order_sorted.begin());
	order_quantities.erase(order_quantities.begin());
	bin_parts.erase(bin_parts.begin());

	return 0;
}

