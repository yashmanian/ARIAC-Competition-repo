/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "move_group_interface_tutorial");
  // ros::NodeHandle node_handle;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // // BEGIN_TUTORIAL
  // //
  // // Setup
  // // ^^^^^
  // //
  // // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // // are used interchangably.
  // static const std::string PLANNING_GROUP = "manipulator";

  // // The :move_group_interface:`MoveGroup` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // // class to add and remove collision objects in our "virtual world" scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const robot_state::JointModelGroup *joint_model_group =
  //   move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // // Getting Basic Information
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // We can print the name of the reference frame for this robot.
  // // ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // // // We can also print the name of the end-effector link for this group.
  // // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // // Planning to a Pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.28;
  // target_pose1.position.y = -0.7;
  // target_pose1.position.z = 1.0;
  // move_group.setPoseTarget(target_pose1);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // move_group.move() 


  // // END_TUTORIAL

  // ros::shutdown();
  // return 0;
}