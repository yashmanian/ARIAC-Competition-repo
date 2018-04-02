#include<ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
//#include <osrf::trajectory_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
//#include <osrf_gear_srv/GetMaterialLocations.h>
//#include <osrf_gear_srv/AGVControl.h>
//#include <osrf_gear_srv/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/AGVControl.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/DisplayTrajectory.h>
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

  /// Create a JointTrajectory with all positions set to zero, and command the arm.

// Order Callback
void order_callback(const osrf_gear::Order::ConstPtr & currentOrder) {
std::vector<osrf_gear::Order> received_order;
received_order.push_back(*currentOrder);
ROS_INFO_STREAM(*currentOrder);
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
while(scan.models.size()==0){
ros::Duration(0.5).sleep();
}
std::vector<osrf_gear::Model> allObjects=scan.models;
for (osrf_gear::Model& object : allObjects){
invent_1.push_back(object);
}
}

void logical_camera_2_callback(
const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
osrf_gear::LogicalCameraImage scan=*image_msg;
gear_numbers=image_msg->models.size();
LogicalImage_2=scan;
while(scan.models.size()==0){
ros::Duration(0.5).sleep();
}
std::vector<osrf_gear::Model> allObjects=scan.models;
for (osrf_gear::Model& object : allObjects){
invent_2.push_back(object);
}
}
};


// Arm controller class
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
    move_group.setPlannerId("TRRTkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(20);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    return move_group;
  }

  void send_arm_to_zero_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3) 
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
    msg.points[0].time_from_start = ros::Duration(2);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
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

  bool dropPart(geometry_msgs::PoseStamped & pose_1a, geometry_msgs::PoseStamped & pose_1b, moveit::planning_interface::MoveGroupInterface & move_group, int order_count)
  {
    pose_1a.pose.orientation.w = w/mag;
    pose_1a.pose.orientation.x = x/mag;
    pose_1a.pose.orientation.y = y/mag;
    pose_1a.pose.orientation.z = z/mag;
    pose_1b.pose.orientation.w = w/mag;
    pose_1b.pose.orientation.x = x/mag;
    pose_1b.pose.orientation.y = y/mag;
    pose_1b.pose.orientation.z = z/mag;
    bool success;
    float lin_a = 0.0;
    float lin_b = 2.1;
    float pan_a = 3.14;
    float pan_b = 1.51;
    float pan_c = 1.4;
    if (order_count%2 != 0)
    {
      lin_b = -lin_b;
      pan_b = 4.71;
      pan_c = 4.6;
    }

    float linear_pos[7] = {lin_a , lin_a, lin_b, lin_b, lin_b, lin_a, lin_a};
    float shoulder_pan[7] = {pan_a, pan_b, pan_b, pan_c, pan_b, pan_b, pan_b};
    float elbow[7] = {1.51, 1.51, 1.51, 1.46, 1.51, 1.51, 1.51};
    float shoulder_lift[7] = {-1.128, -1.128, -1.128, -0.54, -1.128, -1.128, -1.128};
    float wrist1[7] = {3.77, 3.77, 3.77, 3.8, 3.77, 3.77, 3.77};
    float wrist2[7] = {-1.51, -1.51, -1.51, -1.57, -1.51, -1.51, -1.51};
    float wrist3[7] = {0.0, 0.0, 0.0, 1.406, 0.0, 0.0, 0.0};

    int count = 0;

    send_arm_to_zero_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0);
    ros::Duration(5).sleep();

    pose_1a.pose.position.z += 0.17;
    this->armController::goToPose(pose_1a.pose, move_group);
    ros::Duration(3).sleep();

    pose_1a.pose.position.z -= 0.17;
    this->armController::goToPose(pose_1a.pose, move_group);
    this->armController::grab();
    ros::Duration(3).sleep();
    bool gripper_state = isGripperAttached();

    while (gripper_state == true && count < 7)
    {
      ros::spinOnce();
      gripper_state = isGripperAttached();
      send_arm_to_zero_state(elbow[count], linear_pos[count], shoulder_lift[count], shoulder_pan[count], wrist1[count], wrist2[count], wrist3[count]);
      ros::Duration(3).sleep();

      if(count == 3)
      {
        this->armController::release();
        ros::Duration(3).sleep();
      }
      ROS_INFO_STREAM("Count :" << count);
      count++;
    }

    if (count > 4)
    {
      success = true;
      ROS_INFO("Motion complete.");
    }
    else
    {
      success = false;
      ROS_INFO("Part Dropped.");
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



// Main loop
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "qual1_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

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
  armController Arm(node);
  moveit::planning_interface::MoveGroupInterface move_group = Arm.plannerInit();
  bool success = false;

  std::string agvs[]={"agv1_load_point_frame","agv2_load_point_frame"};
  for(osrf_gear::Model inventoryScan: invent_1)
  {
    ROS_INFO_STREAM(inventoryScan.type);
  }


  int currentAgv=-1;
  int order_count = 0;
  for(osrf_gear::Kit currentKit: kitList)
  {
    currentAgv++;
    order_count++;
    std::vector<osrf_gear::KitObject> objectList;
    for(osrf_gear::KitObject& object :currentKit.objects)
    {
      objectList.push_back(object);
    }
    while(!objectList.empty()==true) 
    {
      osrf_gear::KitObject currentOrder=objectList[0];
      osrf_gear::Model Target_Object;
      std::string Target_Bin;
      ROS_INFO_STREAM(currentOrder.type);
      std::vector<osrf_gear::Model> allObjects=LogicalImage_1.models;
      for (osrf_gear::Model& object : allObjects)
      {
        if(object.type==currentOrder.type)
        {
          Target_Object=object;
          Target_Bin="logical_camera_1_frame";
          ROS_INFO_STREAM("logical_camera_1_frame");
          break;
        }
      }

      allObjects=LogicalImage_2.models;
      for (osrf_gear::Model& object : allObjects)
      {
        if(object.type==currentOrder.type)
        {
          Target_Object=object;
          Target_Bin="logical_camera_2_frame";
          ROS_INFO_STREAM("logical_camera_2_frame");
          break;
        }
      }

      geometry_msgs::PoseStamped initialPose, finalPose;
      initialPose.pose=Target_Object.pose;
      finalPose.pose=currentOrder.pose;
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tf2_listener(tfBuffer);
      geometry_msgs::TransformStamped logical_camera_to_world_frame;
      logical_camera_to_world_frame = tfBuffer.lookupTransform( "world", Target_Bin, ros::Time(0), ros::Duration(1.0) );
      tf2::doTransform(initialPose, initialPose, logical_camera_to_world_frame);
      ROS_INFO_STREAM(initialPose);

      geometry_msgs::TransformStamped agv_to_world_frame;
      agv_to_world_frame = tfBuffer.lookupTransform( "world", agvs[currentAgv], ros::Time(0), ros::Duration(1.0) );
      ROS_INFO_STREAM(agvs[currentAgv]);
      tf2::doTransform(finalPose, finalPose, agv_to_world_frame);
      ROS_INFO_STREAM(finalPose);

      float pan_final = 1.51;
      finalPose.pose.position.x = 0.3;
      if (order_count%2 != 0)
      {
        finalPose.pose.position.y = 3.1;
        pan_final = 4.71;
      }
      else
      {
        finalPose.pose.position.y = -3.1;
      }
      
      finalPose.pose.position.z = 0.9;

      bool plan_fin = Arm.dropPart(initialPose, finalPose, move_group, order_count);
      ROS_INFO_STREAM("Motion complete" << plan_fin);

      // objectList.erase(objectList.begin());
      Arm.send_arm_to_zero_state(1.51, 0.0, -1.128, pan_final, 3.77, -1.51, 0.0);
      ros::Duration(2).sleep();

      if (plan_fin == true)
      {
        objectList.erase(objectList.begin());
        Arm.send_arm_to_zero_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0);
        ROS_INFO("Next action.");
      }
      else
      {
        Arm.send_arm_to_zero_state(1.51, 0.0, -1.128, 3.14, 3.77, -1.51, 0.0);
        ros::Duration(2).sleep();
        ROS_INFO("Repeating Action.");
      }

    }
  }
  osrf_gear::AGVControl srv1;
  ros::ServiceClient client1 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  client1.call(srv1);

  osrf_gear::AGVControl srv2;
  ros::ServiceClient client2 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  client1.call(srv2);
  return 0;
}

