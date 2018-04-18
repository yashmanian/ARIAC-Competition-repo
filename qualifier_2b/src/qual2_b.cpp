
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
// #include <osrf::trajectory_msgs/LaserScan.h>
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
//ROS_INFO_STREAM(*currentOrder);
for(osrf_gear::Kit& parseKit :received_order[0].kits){
kitList.push_back(parseKit);
}
}

/// Called when a new LogicalCameraImage message is received.
void logical_camera_1_callback(
const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
osrf_gear::LogicalCameraImage scan=*image_msg;
piston_numbers=image_msg->models.size();
LogicalImage_1=scan;
//while(scan.models.size()==0){
//ros::Duration(0.5).sleep();
//}
//std::vector<osrf_gear::Model> allObjects=scan.models;
//for (osrf_gear::Model& object : allObjects){
//invent_1.push_back(object);
//}
}

//osrf_gear::LogicalCameraImage scanShopFloor(){
//osrf_gear::LogicalCameraImage Image=Logical_Camera_Data();
//return Image;
//}

};




int main(int argc, char ** argv){
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
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_1_callback, &comp_class);

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
ros::Rate rate(10.0);

  ROS_INFO("Setup complete.");
  start_competition(node);
 ros::Duration(0.5).sleep();
 armController Arm(node);
//while(ros::ok()){
ros::spinOnce();  // This executes callbacks on new data until ctrl-c.
//for(osrf_gear::KitObject object: objectList){
//ROS_INFO_STREAM("Object"<<object.type);
//}
//for(osrf_gear::Model object_1: invent){
//ROS_INFO_STREAM("Object"<<object_1.pose.position.x);
//}

std::string agvs[]={"agv1_load_point_frame","agv2_load_point_frame"};
//for(osrf_gear::Model inventoryScan: invent_1){
//ROS_INFO_STREAM(inventoryScan.type);
//}
ROS_INFO_STREAM("Processing Order");
int currentAgv=-1;
for(osrf_gear::Kit currentKit: kitList){
	ROS_INFO_STREAM("Processing Kit");
currentAgv++;
std::vector<osrf_gear::KitObject> objectList;
for(osrf_gear::KitObject& object :currentKit.objects){
objectList.push_back(object);
}

Arm.set_joint_state(1.65, 0.0, -0.73, 0.0, 3.77, -1.51, 0.0, 1.0);
ros::Duration(2).sleep();
Arm.go_to_belt();
ros::Duration(2).sleep();
while(!objectList.empty()==true) {
	// Add go-to-home
  Arm.go_to_belt();
  ros::Duration(2).sleep();
	int orderCase=0;
	bool found=false;
	osrf_gear::Model Target_Object;
	std::string Target_Bin;
	ros::spinOnce();
	while(found==false)
  {
	std::vector<osrf_gear::Model> allObjects=LogicalImage_1.models;
	for (osrf_gear::Model& detectedObject : allObjects)
  {
    orderCase = 0;
		for(osrf_gear::KitObject& orderPart : objectList)
    {
			if(detectedObject.type==orderPart.type)
      {
			Target_Object=detectedObject;
			Target_Bin="logical_camera_1_frame";
			ROS_INFO_STREAM("logical_camera_1_frame");
			found=true;
			// Add pick up the part
      if (Target_Object.type == "disk_part")
      {
        Arm.grabPart_belt1();
      }
      else if (Target_Object.type == "gasket_part")
      {
        Arm.grabPart_belt2();
      }
      
      ros::spinOnce();
      Arm.go_to_agv1();
      Arm.release();

			break;
		}
		else {
			orderCase++;
		}
		if(found==true){
			break;
		}
		}
		if(found==true){
			break;
		}
		}
	ros::spinOnce();
	}
	ros::spinOnce();
	//geometry_msgs::PoseStamped initialPose, finalPose;
	//initialPose.pose=Target_Object.pose;
	// auto currentOrder=objectList.at(0+orderCase);
	// ROS_INFO_STREAM(orderCase);
	//finalPose.pose=currentOrder.pose;
	//tf2_ros::Buffer tfBuffer;
	//tf2_ros::TransformListener tf2_listener(tfBuffer);
	//geometry_msgs::TransformStamped logical_camera_to_world_frame;
	//logical_camera_to_world_frame = tfBuffer.lookupTransform( "world", Target_Bin, ros::Time(0), ros::Duration(1.0) );
	//tf2::doTransform(initialPose, initialPose, logical_camera_to_world_frame);
	//ROS_INFO_STREAM(initialPose);
	//geometry_msgs::TransformStamped agv_to_world_frame;
	//agv_to_world_frame = tfBuffer.lookupTransform( "world", agvs[currentAgv], ros::Time(0), ros::Duration(1.0) );
	//ROS_INFO_STREAM(agvs[currentAgv]);
	//tf2::doTransform(finalPose, finalPose, agv_to_world_frame);
	//ROS_INFO_STREAM(finalPose);
	objectList.erase(objectList.begin()+orderCase);
}
}
return 0;
}

