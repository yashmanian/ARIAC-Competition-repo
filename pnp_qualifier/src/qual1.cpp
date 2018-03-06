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



// Global Variables
std::vector<osrf_gear::KitObject> objectList;
std::vector<osrf_gear::Model> invent;
osrf_gear::LogicalCameraImage LogicalImage;
std::string logical_camera_frame;
std::string part_frame;
std::string part_frame_no;
int piston_id=1;
int gear_id=1;


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
//while(ros::ok()){
ros::spinOnce();  // This executes callbacks on new data until ctrl-c.
//for(osrf_gear::KitObject object: objectList){
//ROS_INFO_STREAM("Object"<<object.type);
//}
//for(osrf_gear::Model object_1: invent){
//ROS_INFO_STREAM("Object"<<object_1.pose.position.x);
//}

for(osrf_gear::Model inventoryScan: invent){
ROS_INFO_STREAM(inventoryScan.type);
}


while(!objectList.empty()==true) {
osrf_gear::KitObject currentOrder=objectList[0];
ROS_INFO_STREAM(currentOrder.type);
//ROS_INFO_STREAM("Object"<< currentOrder.type);
//for(osrf_gear::Model inventoryScan: invent){
//if((inventoryScan.type).compare(currentOrder.type)==0){
//ROS_INFO_STREAM("Found "<<inventoryScan.type<<" at "<<inventoryScan.pose.position.x<<"Logical Camera at"<<LogicalImage.pose
//		<<" Goal_Position "<< currentOrder.pose.position.x);
if(currentOrder.type=="piston_rod_part"){
logical_camera_frame="logical_camera_1_";
part_frame="piston_rod_part_";
part_frame_no=part_frame+std::to_string(piston_id)+"_frame";
piston_id++;
                                         } else{
logical_camera_frame="logical_camera_2_";
part_frame="gear_part_";
part_frame_no=part_frame+std::to_string(gear_id)+"_frame";
gear_id++;
}
bool done=false;
while (done==false){
geometry_msgs::TransformStamped transformStamped;
    try{
ros::Time now = ros::Time::now();
      transformStamped = tfBuffer.lookupTransform("world", logical_camera_frame+part_frame_no,
                               ros::Time(0),ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }ROS_INFO_STREAM("Transform"<<transformStamped);
done=true;
geometry_msgs::Pose poseData;
poseData.position.x=transformStamped.transform.translation.x;
poseData.position.y=transformStamped.transform.translation.y;
poseData.position.z=transformStamped.transform.translation.z;
poseData.orientation.x=transformStamped.transform.rotation.x;
poseData.orientation.y=transformStamped.transform.rotation.y;
poseData.orientation.z=transformStamped.transform.rotation.z;
poseData.orientation.w=transformStamped.transform.rotation.w;
partLocation.publish(poseData);
ROS_INFO_STREAM("start: "<<poseData);
}

geometry_msgs::Pose goalData;
goalData=currentOrder.pose;
partLocation.publish(goalData);
ROS_INFO_STREAM("goal: "<<goalData);

// Enter ArmController Call here.
// Enter faulty part test here.
//ros::spinOnce();
//}
//break;
//}
objectList.erase(objectList.begin());
}
//}
return 0;
}
