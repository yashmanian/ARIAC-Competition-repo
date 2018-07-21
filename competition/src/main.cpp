/**
* @author Vaibhav Bhilare
* @copyright 2018, Vaibhav Bhilare
*/


#include "../include/manager.h"




int main(int argc, char ** argv){
  ros::init(argc, argv, "competition_node");
  ros::NodeHandle node;
  manager manager_class(node);
  // Subscribe to the '/ariac/current_score' topic.


  // ROS_INFO_STREAM("Processing Order");
  // int currentAgv=-1;
  // for(osrf_gear::Kit currentKit: manager_class.kitList) {
  //   ROS_INFO_STREAM("Processing Kit");
  //   currentAgv++;
  //   std::vector<osrf_gear::KitObject> objectList;
  //   for(osrf_gear::KitObject& object :currentKit.objects){
  //     objectList.push_back(object);
  //   }
  //   while(!objectList.empty()==true) {
  //   // Add go-to-home
  //     ros::Duration(2).sleep();
  //     int orderCase=0;
  //     bool found=false;
  //     osrf_gear::Model Target_Object;
  //     std::string Target_Bin;
  //     ros::spinOnce();
  //     while(found==false) {
  //       std::vector<osrf_gear::Model> allObjects=manager_class.LogicalImage_1.models;
  //       for (osrf_gear::Model& detectedObject : allObjects) {
  //         orderCase = 0;
  //         for(osrf_gear::KitObject& orderPart : objectList) {
  //           if(detectedObject.type==orderPart.type) {
  //             Target_Object=detectedObject;
  //             Target_Bin="logical_camera_1_frame";
  //             ROS_INFO_STREAM("logical_camera_1_frame");
  //             found=true;
  //             // Add pick up the part
  //             ros::spinOnce();
  //             break;
  //           }
  //           else {
  //             orderCase++;
  //           }
  //           if(found==true){
  //             break;
  //           }
  //         }
  //         if(found==true){
  //           break;
  //         }
  //       }
  //       ros::spinOnce();
  //     }
  //     ros::spinOnce();
  //     //geometry_msgs::PoseStamped initialPose, finalPose;
  //     //initialPose.pose=Target_Object.pose;
  //     // auto currentOrder=objectList.at(0+orderCase);
  //     // ROS_INFO_STREAM(orderCase);
  //     //finalPose.pose=currentOrder.pose;
  //     //tf2_ros::Buffer tfBuffer;
  //     //tf2_ros::TransformListener tf2_listener(tfBuffer);
  //     //geometry_msgs::TransformStamped logical_camera_to_world_frame;
  //     //logical_camera_to_world_frame = tfBuffer.lookupTransform( "world", Target_Bin, ros::Time(0), ros::Duration(1.0) );
  //     //tf2::doTransform(initialPose, initialPose, logical_camera_to_world_frame);
  //     //ROS_INFO_STREAM(initialPose);
  //     //geometry_msgs::TransformStamped agv_to_world_frame;
  //     //agv_to_world_frame = tfBuffer.lookupTransform( "world", agvs[currentAgv], ros::Time(0), ros::Duration(1.0) );
  //     //ROS_INFO_STREAM(agvs[currentAgv]);
  //     //tf2::doTransform(finalPose, finalPose, agv_to_world_frame);
  //     //ROS_INFO_STREAM(finalPose);
  //     objectList.erase(objectList.begin()+orderCase);
  //   }
  // }
  return 0;
}