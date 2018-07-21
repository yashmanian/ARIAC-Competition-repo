//
//	Created by Yash Manian on 04/10/2018
//

#include "../include/armController.h" 

// Constructor class
armController::armController(ros::NodeHandle & node)
{
  joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
  gripperStateSubscriber = node.subscribe("/ariac/gripper/state", 10, &armController::gripperStateCallback, this);
  gripper_client = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
  beam_1_StateSubscriber = node.subscribe("/ariac/break_beam_1", 1, &armController::beam_1_Callback, this);
  beam_2_StateSubscriber = node.subscribe("/ariac/break_beam_2", 1, &armController::beam_2_Callback, this);
  beam_3_StateSubscriber = node.subscribe("/ariac/break_beam_3", 1, &armController::beam_3_Callback, this);
  beam_4_StateSubscriber = node.subscribe("/ariac/break_beam_4", 1, &armController::beam_4_Callback, this);
  joint_state_subscriber = node.subscribe("/ariac/joint_states", 1, &armController::joint_state_callback, this);
}


// Initialize planner
moveit::planning_interface::MoveGroupInterface armController::plannerInit()
{
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlannerId("RRTkConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(20);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  return move_group;
}

// Print joint states
void armController::printStates()
{
  ROS_INFO_STREAM("Current elbow: " << curr_elbow);
  ROS_INFO_STREAM("Current y_pos: " << curr_y_pos);
  ROS_INFO_STREAM("Current lift: " << curr_sh_lift);
  ROS_INFO_STREAM("Current pan: " << curr_sh_pan);
  ROS_INFO_STREAM("Current w1: " << curr_wrist_1);
  ROS_INFO_STREAM("Current w2: " << curr_wrist_2);
  ROS_INFO_STREAM("Current w3: " << curr_wrist_3);
}

// Set joint states
void armController::set_joint_state(float elbow, float linear_pos, float shoulder_lift, float shoulder_pan, float wrist1, float wrist2, float wrist3, float time) 
{
  trajectory_msgs::JointTrajectory msg;
  msg.joint_names.clear();
  msg.joint_names.push_back("elbow_joint");
  msg.joint_names.push_back("linear_arm_actuator_joint");
  msg.joint_names.push_back("shoulder_lift_joint");
  msg.joint_names.push_back("shoulder_pan_joint");
  msg.joint_names.push_back("wrist_1_joint");
  msg.joint_names.push_back("wrist_2_joint");
  msg.joint_names.push_back("wrist_3_joint");

  curr_elbow = elbow;
  curr_y_pos = linear_pos;
  curr_sh_lift = shoulder_lift;
  curr_sh_pan = shoulder_pan;
  curr_wrist_1 = wrist1;
  curr_wrist_2 = wrist2;
  curr_wrist_3 = wrist3;

  msg.points.resize(1);

  msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
  msg.points[0].positions[0] = elbow;
  msg.points[0].positions[1] = linear_pos;
  msg.points[0].positions[2] = shoulder_lift;
  msg.points[0].positions[3] = shoulder_pan;
  msg.points[0].positions[4] = wrist1;
  msg.points[0].positions[5] = wrist2;
  msg.points[0].positions[6] = wrist3;

  msg.points[0].time_from_start = ros::Duration(time);
  joint_trajectory_publisher_.publish(msg);
}


/*------------------ Motion methods ------------------*/
void armController::go_to_home()
{
  ROS_INFO("Moving to home");
  printStates();
  // set_joint_state(2.83, 0.62, -1.72, 3.31, 3.60, -1.51, 0.14, 1);

  set_joint_state(2.6873, 0.3858, -1.9156, 3.4361, 2.5531, -1.9382, 0.0987, 1);
  ros::Duration(1).sleep();
  ROS_INFO("Moved to home");
}

void armController::go_to_agv1()
{
  ROS_INFO("Moving to AGV1");
  set_joint_state(2.83, curr_y_pos, -1.72, curr_sh_pan, 3.60, -1.51, 0.14, 0.7);
  ros::Duration(0.7).sleep();

  set_joint_state(curr_elbow, curr_y_pos, curr_sh_lift, -4.71, curr_wrist_1, curr_wrist_2, curr_wrist_3, 0.3);
  ros::Duration(0.5).sleep();

  printStates();
  set_joint_state(1.46, 2.1, -0.54, -4.71, 3.8, -1.57, 1.406, 0.8);
  ros::Duration(0.8).sleep();
  ROS_INFO("Moved to AGV1");
}

void armController::go_to_agv2()
{
  ROS_INFO("Moving to AGV2");
  printStates();
  set_joint_state(2.83, curr_y_pos, -1.72, curr_sh_pan, 3.60, -1.51, 0.14, 0.7);
  ros::Duration(0.7).sleep();

  set_joint_state(curr_elbow, curr_y_pos, curr_sh_lift, 4.71, curr_wrist_1, curr_wrist_2, curr_wrist_3, 0.5);
  ros::Duration(0.5).sleep();

  set_joint_state(1.46, -2.1, -0.54, 4.71, 3.8, -1.57, 1.406, 0.8);
  ros::Duration(0.8).sleep();
  ROS_INFO("Moved to AGV2");
}

void armController::travel_left()
{
  printStates();
  set_joint_state(1.51, 0.0, -1.128, 1.57, 3.77, -1.51, 0.0, 1.5);
  ROS_INFO("Travelling left");
}

void armController::travel_right()
{
  printStates();
  set_joint_state(1.51, 0.0, -1.128, 4.71, 3.77, -1.51, 0.0, 1.5);
  ros::Duration(1).sleep();
  ROS_INFO("Travelling right");
}

void armController::go_to_belt()
{
  printStates();
  set_joint_state(1.65, 2.0, -0.73, 0.0, 3.77, -1.51, 0.0, 0.8);
  ros::Duration(0.8).sleep();
  ROS_INFO("Moved to belt");
}

void armController::go_to_bin(int bin_count)
{
  ROS_INFO_STREAM("Going to bin " << bin_count);
  float y_pos = 0;
  if (bin_count == 5)
  {
    y_pos = -1.08;
  }
  else  if (bin_count == 6)
  {
    y_pos = -0.30;
  }
  else if (bin_count == 7)
  {
    y_pos = 0.43;
  }
  else if (bin_count == 8)
  {
    y_pos = 1.195;
  }
  set_joint_state(2.6873, curr_y_pos, -1.9156, curr_sh_pan, 2.5531, -1.9382, 0.0987, 0.5);
  ros::Duration(0.8).sleep();

  set_joint_state(curr_elbow, y_pos, curr_sh_lift, curr_sh_pan, curr_wrist_1, curr_wrist_2, curr_wrist_3, 0.5);
  ros::Duration(0.5).sleep();

  set_joint_state(2.17, y_pos, -1.063, 3.262, 3.589, -1.520, 0.090, 0.8);
  ros::Duration(0.8).sleep();
  ROS_INFO_STREAM("Moved to bin " << bin_count);
}

void armController::dropPose(float pos_x, float pos_y, int agv_count)
{
  int select;
  int row = 1;
  int col = 1;
  float row_cmp_a = 0;
  float row_cmp_b = 0;
  float pan_angle1 = 1.570;
  float pan_angle2 = 4.71;
  float y_pose1 = 2.1;
  float y_pose2 = -2.1;
  float base_y;
  float base_pan;


  if (pos_x < 0)
  {
    float a = abs(pos_x) - 0.15;
    float b = abs(pos_x) - 0.1;

    if (a < 0 && b > 0.0001)
    {
      col = 1;
    }
    else if(a > 0 && a < 0.001)
    {
      col = 1;
    }
    else if(b>0 && b < 0.0001)
    {
      col = 2;
    }
    else if(b<0)
    {
      col = 2;
    }
  }

  else if (pos_x == 0)
  {
    col = 3;
  }

  else if(pos_x > 0)
  {
    float a = pos_x - 0.15;
    float b = pos_x - 0.1;
    if (a < 0 && b > 0.0001)
    {
      col = 5;
    }
    else if(a > 0 && a < 0.001)
    {
      col = 5;
    }
    else if(b>0 && b < 0.0001)
    {
      col = 4;
    }
    else if(b<0)
    {
      col = 4;
    }
  }


  if (agv_count == 1)
  {
    select = -1;
    base_y = y_pose1;
    base_pan = pan_angle1;
  }
  else if (agv_count == 2)
  {
    select = 1;
    base_y = y_pose2;
    base_pan = pan_angle2;
  }

  float y_pose = base_y + select*(pos_y + 0.2);
  float pan_angle[5] = {base_pan + delta_pan[0], base_pan  + delta_pan[1], base_pan  + delta_pan[2], base_pan + delta_pan[3], base_pan + delta_pan[4]};
  
  drop_col = col;
  drop_y_pose = y_pose;

  set_joint_state(dep_elbow[col-1], y_pose, dep_sh_lift[col - 1], pan_angle[col-1], dep_w1[col-1], dep_w2[col-1], dep_w3[col-1], 0.4);
  ros::Duration(0.5).sleep();
}


void armController::FaultyPartPose(string part_type, int agv_count)
{
  float pan_angle1 = 1.570;
  float pan_angle2 = 4.71;
  int col = drop_col;
  float curr_drop_y = drop_y_pose;
  int select;
  float base_pan;
  if (agv_count == 1)
  {
    select = -1;
    base_pan = pan_angle1;
  }
  else if (agv_count == 2)
  {
    select = 1;
    base_pan = pan_angle2;
  }
  float pan_angle[5] = {base_pan + delta_pan[0], base_pan  + delta_pan[1], base_pan  + delta_pan[2], base_pan + delta_pan[3], base_pan + delta_pan[4]};

  set_joint_state(dep_elbow[col-1], curr_drop_y, dep_sh_lift[col - 1], pan_angle[col-1], dep_w1[col-1], dep_w2[col-1], dep_w3[col-1], 0.41);
  ros::Duration(0.5).sleep();

  if (part_type == "pulley_part")
  {

    set_joint_state(faulty_elbow_pulley[col-1], curr_drop_y, faulty_sh_lift_pulley[col-1], pan_angle[col-1], faulty_w1_pulley[col-1], faulty_w2_pulley[col-1], faulty_w2_pulley[col-1], 1);
  }
  else
  {
    set_joint_state(faulty_elbow[col-1], curr_drop_y, faulty_sh_lift[col-1], pan_angle[col-1], faulty_w1[col-1], faulty_w2[col-1], faulty_w2[col-1], 0.4);
  }
  ros::Duration(0.5).sleep();
}


bool armController::grabFaultyPart(string part_type, int agv_count)
{
  bool success = false;
  int select;
  float pan_faulty;
  bool gripperState;

  if (agv_count == 1)
  {
    select = 1;
    pan_faulty = 1.57;
  }
  else if(agv_count == 2)
  {
    select = -1;
    pan_faulty = 4.71;
  }
  float delta_y = 0.5;

  set_joint_state(2.6873, curr_y_pos, -1.9156, pan_faulty, 3.8, -1.57, 1.406, 0.4);
  ros::Duration(0.5).sleep();

  FaultyPartPose(part_type, agv_count);
  grab();
  ros::Duration(0.8).sleep();
  ROS_INFO("Grab Initiated");

  ros::spinOnce();
  gripperState = isGripperAttached();
  while (gripperState == true)
  {
    ROS_INFO("Faulty Grabbed");
    set_joint_state(1.46, curr_y_pos, -0.54, curr_sh_pan, 3.8, -1.57, 1.406, 1);
    ros::Duration(1).sleep();

    ros::spinOnce();
    gripperState = isGripperAttached();
    if (gripperState == false)
    {
      ROS_INFO("Faulty pickup failed");
      break;
    }

    // set_joint_state(curr_elbow, curr_y_pos - select*delta_y, -0.7, curr_sh_pan, curr_wrist_1, curr_wrist_2, curr_wrist_3, 1);
    set_joint_state(2.83, curr_y_pos, -1.72, curr_sh_pan, 3.60, -1.51, 0.14, 0.5);
    ros::Duration(0.5).sleep();

    release();
    success = true;
  }

  return success;
}



bool armController::grabBeltPart(int beam_count, string part_type, float velocity)
{
  ROS_INFO("Initiallizing part grab");
  printStates();
  bool beam_select;
  bool success;
  float drop_factor = def_vel/abs(velocity);

  ROS_INFO_STREAM("Drop factor: " << drop_factor);
  float y_position = (beam_count - 1)* 0.65;
  int loop = 0;

  ros::spinOnce();

  set_joint_state(curr_elbow, curr_y_pos, curr_sh_lift - 0.2, 0.0, curr_wrist_1, curr_wrist_2, curr_wrist_3, 0.5);
  ros::Duration(0.5).sleep();
  if (part_type == "pulley_part")
  {
    set_joint_state(1.65, y_position, -0.75, 0.0, 3.77, -1.51, 0.0, 1);
    ros::Duration(1).sleep();
  }
  else
  {
    set_joint_state(1.65, y_position, -0.73, 0.0, 3.77, -1.51, 0.0, 1);
    ros::Duration(1).sleep();
  }

  // ROS_INFO("Approached belt pos");

  while (loop == 0)
  {
    if (beam_count == 1)
    {
      beam_select = beamBreak1;
      // ROS_INFO_STREAM("Beam 1 state: " << beam_select);
    }
    else if (beam_count == 2)
    {
      beam_select = beamBreak2;
      // ROS_INFO_STREAM("Beam 2 state: " << beam_select);
    }
    else if (beam_count == 3)
    {
      beam_select = beamBreak3;
      // ROS_INFO_STREAM("Beam 3 state: " << beam_select);
    }
    else if (beam_count == 4)
    {
      beam_select = beamBreak4;
      // ROS_INFO_STREAM("Beam 4 state: " << beam_select);
    }

    // ROS_INFO_STREAM("Beam break 3: "<< beam_select);
    if (beam_select == true)
    {
      
      // ROS_INFO("Beam tripped"); 
      if (part_type == "gasket_part")
      {
        
        set_joint_state(1.67, y_position, -0.65, 0.0, 3.73, -1.57, 0.049, drop_factor*0.8);
        ROS_INFO_STREAM("Part factor: " << drop_factor*0.8);
      }
      else if (part_type == "disk_part")
      {
        set_joint_state(1.67, y_position, -0.65, 0.0, 3.73, -1.57, 0.049, drop_factor*0.8);
        ROS_INFO_STREAM("Part factor: " << drop_factor*0.95);
      }
      else if (part_type == "piston_rod_part")
      {
        set_joint_state(1.656, y_position, -0.656, 0.0011, 3.716, -1.5699, 0.0, drop_factor*0.16);
        ROS_INFO_STREAM("Part factor: " << drop_factor*0.16);
      }
      else if (part_type == "gear_part")
      {
        set_joint_state(1.656, y_position, -0.66, 0.0011, 3.716, -1.5699, 0.0, drop_factor*0.19);
        ROS_INFO_STREAM("Part factor: " << drop_factor*0.15);
      }
      else if (part_type == "pulley_part")
      {
        set_joint_state(1.6692, y_position, -0.7454, 0.0043, 3.7919, -1.5698, 0.0044, drop_factor*0.2);
        ROS_INFO_STREAM("Part factor: " << drop_factor*0.2);
      }


      while (beam_select == true)
      {
        ros::spinOnce();
        grab();
        bool gripperState = isGripperAttached();
        if (gripperState == true)
        {
          break;
        } 
      }
      set_joint_state(2.83, y_position, -1.72, 0.0, 3.77, -1.51, 0.0, 0.1);
      ros::Duration(0.5).sleep();
      ROS_INFO("Grabbed");
      success = true;
      loop = 1;
    }
  }

  return success;
}


bool armController::depositPart(int agv_count, geometry_msgs::Pose finalPose, moveit::planning_interface::MoveGroupInterface & move_group, string part_type)
{
  float pan_angle = 0;
  float return_pan_angle = 0;
  bool success = false;
  if (agv_count == 1)
  {
    pan_angle = -4.71;
    return_pan_angle = 4.71;
  }
  else if (agv_count == 2)
  {
    pan_angle = 4.71;
    return_pan_angle = -4.71;
  }

  if (part_type == "piston_rod_part")
  {
    finalPose.position.z = 0.83;
  }
  else if (part_type == "gear_part")
  {
    finalPose.position.z = 0.83;
  }
  else if (part_type == "disk_part")
  {
    finalPose.position.z = 0.83;
  }
  else if (part_type == "gasket_part")
  {
    finalPose.position.z = 0.83;
  }
  else if (part_type == "pulley_part")
  {
    finalPose.position.z = 0.93;
  }

  ros::spinOnce();
  bool gripperState = isGripperAttached();
  while (gripperState == true)
  {
    set_joint_state(curr_elbow, curr_y_pos, curr_sh_lift, pan_angle, curr_wrist_1, curr_wrist_2, curr_wrist_3, 1);
    ros::Duration(1).sleep();
    ros::spinOnce();
    gripperState = isGripperAttached();

    if (agv_count == 1)
    {
      go_to_agv1();
      ros::spinOnce();
      // ros::Duration(1).sleep();
      ros::spinOnce();
      gripperState = isGripperAttached();
    }
    else if (agv_count == 2)
    {
      go_to_agv2();
      
      // ros::Duration(1).sleep();
      ros::spinOnce();
      gripperState = isGripperAttached();
      if (gripperState == false)
      {
        break;
      }
    }

    dropPose(finalPose.position.x, finalPose.position.y, agv_count);
    ros::Duration(0.5).sleep();
    // bool poseSuccess = false;

    // while(poseSuccess == false)
    // {
    //   ros::spinOnce();
    //   gripperState = isGripperAttached();
    //   if (gripperState == false)
    //   {
    //     break;
    //   }
    //   poseSuccess = goToPose(finalPose, move_group);
    //   ros::Duration(1.5).sleep();
    // }

    // ROS_INFO_STREAM("Status: " << poseSuccess);

    ros::spinOnce();
    gripperState = isGripperAttached();
    if (gripperState == false)
    {
      break;
    }

    release();
    ros::Duration(0.4).sleep();
    success = true;
    ros::spinOnce();
    gripperState = isGripperAttached();
    if (gripperState == false)
    {
      break;
    }
    
  }
  ros::Duration(0.2).sleep();

  set_joint_state(2.83, curr_y_pos, -1.72, curr_sh_pan, 3.60, -1.51, 0.14, 0.5);
  ros::Duration(0.5).sleep();
  ROS_INFO_STREAM("Status: " << success);
  return success;
}


bool armController::grabBinPart(int bin_count, string part_type)
{
  ros::spinOnce();

  // target_pose.orientation.w = w/mag;
  // target_pose.orientation.x = x/mag;
  // target_pose.orientation.y = y/mag;
  // target_pose.orientation.z = z/mag;

  // if (part_type == "piston_rod_part")
  // {
  //   target_pose.position.z = 0.729;
  // }
  // else if (part_type == "gear_part")
  // {
  //   target_pose.position.z = 0.729;
  // }
  // else if (part_type == "disk_part")
  // {
  //   target_pose.position.z = 0.729;
  // }
  // else if (part_type == "gasket_part")
  // {
  //   target_pose.position.z = 0.729;
  // }
  // else if (part_type == "pulley_part")
  // {
  //   target_pose.position.z = 0.80;
  // }

  go_to_bin(bin_count);
  float y_bin = curr_y_pos;
  float y_diff = 0;
  float delta_y = 0.10;

  int count_row = 1;
  int count_col = 1;
  int max_col = 6;
  int max_row = 4;

  if (part_type == "pulley_part")
  {
    max_col = 2;
    delta_y = 0.3;
  }
  if (part_type == "gasket_part")
  {
    max_col = 4;
    delta_y = 0.135;
  }

  ros::spinOnce();
  bool success = isGripperAttached();

  while (success == false)
  {
    if (count_row > max_row)
    {
      ROS_INFO("No Parts Grabbed");
      break;
    }

    if (part_type == "pulley_part")
    {
      y_diff = y_bin + y_offset_pulley[count_row-1] - delta_y*(count_col - 1);
      set_joint_state(trans_elbow_pulley[count_row-1], y_diff, trans_sh_lift_pulley[count_row-1], trans_sh_pan_pulley[count_row-1], trans_w1_pulley[count_row-1], trans_w2_pulley[count_row-1], trans_w3_pulley[count_row-1], 0.1);
    }
    else
    {
      y_diff = y_bin + y_offset_pulley[count_row-1] - delta_y*(count_col - 1);
      set_joint_state(trans_elbow[count_row-1], y_diff, trans_sh_lift[count_row-1], trans_sh_pan[count_row-1], trans_w1[count_row-1], trans_w2[count_row-1], trans_w3[count_row-1], 0.1);
    }
    ros::Duration(0.2).sleep();


    if (part_type == "pulley_part")
    {
      y_diff = y_bin + y_offset_pulley[count_row-1] - delta_y*(count_col - 1);
      set_joint_state(pick_elbow_pulley[count_row-1], y_diff, pick_sh_lift_pulley[count_row-1], pick_sh_pan_pulley[count_row-1], pick_w1_pulley[count_row-1], pick_w2_pulley[count_row-1], pick_w3_pulley[count_row-1], 0.1);
    }
    else
    {
      y_diff = y_bin + y_offset[count_row-1] - delta_y*(count_col - 1);
      set_joint_state(pick_elbow[count_row-1], y_diff, pick_sh_lift[count_row-1], pick_sh_pan[count_row-1], pick_w1[count_row-1], pick_w2[count_row-1], pick_w3[count_row-1], 0.1);
    }

    ros::Duration(0.2).sleep();
    grab();
    ros::Duration(0.4).sleep();

    success = isGripperAttached();
    if (success == true)
    {
      ROS_INFO("Grabbed Part");
      break;
    }

    if (part_type == "pulley_part")
    {
      set_joint_state(trans_elbow_pulley[count_row-1], y_diff, trans_sh_lift_pulley[count_row-1], trans_sh_pan_pulley[count_row-1], trans_w1_pulley[count_row-1], trans_w2_pulley[count_row-1], trans_w3_pulley[count_row-1], 0.1);
    }
    else
    {
      set_joint_state(trans_elbow[count_row-1], y_diff, trans_sh_lift[count_row-1], trans_sh_pan[count_row-1], trans_w1[count_row-1], trans_w2[count_row-1], trans_w3[count_row-1], 0.1);
    }
    ros::Duration(0.2).sleep();

    if (count_col < max_col)
    {
      count_col++;
      ROS_INFO_STREAM("Moving to next column: " << count_col);
      ROS_INFO_STREAM("Y value: " << y_diff);
    }
    else if(count_col == max_col)
    {
      count_col = 1;
      count_row++;
      ROS_INFO_STREAM("Moving to next row: " << count_row);
    }
  }
  // target_pose.position.z = 0.83;
  // goToPose(target_pose, move_group);
  // ros::Duration(1).sleep();

  // target_pose.position.z = 0.73;
  // goToPose(target_pose, move_group);
  // this->armController::grab();
  // ros::Duration(1.5).sleep();  

  ros::spinOnce();
  success = isGripperAttached();
  if (success == true)
  {
    if (part_type == "pulley_part")
    {
      set_joint_state(trans_elbow_pulley[count_row-1], y_diff, trans_sh_lift_pulley[count_row-1], trans_sh_pan_pulley[count_row-1], trans_w1_pulley[count_row-1], trans_w2_pulley[count_row-1], trans_w3_pulley[count_row-1], 0.1);
    }
    else
    {
      set_joint_state(trans_elbow[count_row-1], y_diff, trans_sh_lift[count_row-1], trans_sh_pan[count_row-1], trans_w1[count_row-1], trans_w2[count_row-1], trans_w3[count_row-1], 0.1);
    }
    ros::Duration(0.2).sleep();

    go_to_bin(bin_count);
  }
  
  set_joint_state(2.83, curr_y_pos, -1.72, curr_sh_pan, 3.60, -1.51, 0.14, 0.5);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  success = isGripperAttached();
  ROS_INFO_STREAM(success);

  

  return success;
}


/*------------------ Callback methods ------------------*/

void armController::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) 
{
  currentGripperState = *msg;
  attached = msg->attached;
}


void armController::beam_1_Callback(const osrf_gear::Proximity::ConstPtr &msg)
{
  osrf_gear::Proximity beamState = *msg;
  beamBreak1 = msg->object_detected;
  //ROS_INFO_STREAM("Break beam 1:\n" << beamBreak1);
}

void armController::beam_2_Callback(const osrf_gear::Proximity::ConstPtr &msg)
{
  osrf_gear::Proximity beamState = *msg;
  beamBreak2 = msg->object_detected;
  //ROS_INFO_STREAM("Break beam 2:\n" << beamBreak2);
}

void armController::beam_3_Callback(const osrf_gear::Proximity::ConstPtr &msg)
{
  osrf_gear::Proximity beamState = *msg;
  beamBreak3 = msg->object_detected;
  // ROS_INFO_STREAM("Break beam 3: " << beamBreak3);
}

void armController::beam_4_Callback(const osrf_gear::Proximity::ConstPtr &msg)
{
  osrf_gear::Proximity beamState = *msg;
  beamBreak4 = msg->object_detected;
  //ROS_INFO_STREAM("Break beam 4:\n" << beamBreak4);
}


/*------------------ Vacuum Gripper methods ------------------*/
void armController::joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
  current_joint_states_ = *joint_state_msg;
}


osrf_gear::VacuumGripperState armController::getGripperState() 
{
  ros::spinOnce();
  return currentGripperState;
}

bool armController::isGripperAttached() 
{
  ros::spinOnce();
  return attached;
}

void armController::grab() 
{
  //ROS_INFO("Enable gripper");
  attach_.request.enable = 1;
  gripper_client.call(attach_);
  ros::spinOnce();
  //ROS_INFO("I %s got the part", isGripperAttached()? "have": "haven't");
}

void armController::release() 
{
  ROS_INFO("Release gripper");
  attach_.request.enable = 0;
  gripper_client.call(attach_);
  ros::spinOnce();
}


/*------------------ Moveit! methods ------------------*/
bool armController::goToPose(geometry_msgs::Pose & target_pose, moveit::planning_interface::MoveGroupInterface & move_group)
{
  move_group.setPoseTarget(target_pose);
  move_group.plan(my_plan);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  return success;
}
