// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdlib>
// TM Driver header
#include "tm_msgs/FeedbackState.h"
#include <fstream>
std::fstream file; 
void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  std::string s = std::to_string(ros::Time::now().toNSec()) + ",";
  
  for (int i = 0; i < 6; i++){
    s += std::to_string(msg->joint_pos[i]) + ",";
  }
  for (int i = 0; i < 6; i++){
    s += std::to_string(msg->joint_vel[i]) + ",";
  }
  for (int i = 0; i < 6; i++){
    s += std::to_string(msg->joint_tor[i]) + ",";
  }
  for (int i = 0; i < 6; i++){
    s += std::to_string(msg->tool_pose[i]) + ",";
  }
  for (int i = 0; i < 5; i++){
    s += std::to_string(msg->tcp_speed[i]) + ",";
  }
  s += std::to_string(msg->tcp_speed[5]) + "\n";
  file << s;
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FeedbackState");
  ros::NodeHandle nh_demo_get;
  file.open("FK_tm5_700_state.csv", std::ios::out | std::ios::trunc);
  file << "time_stamp,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,\
              joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,\
              joint_tor_1,joint_tor_2,joint_tor_3,joint_tor_4,joint_tor_5,joint_tor_6,\
              tool_pose_1,tool_pose_2,tool_pose_3,tool_pose_4,tool_pose_5,tool_pose_6,\
              tcp_speed_1,tcp_speed_2,tcp_speed_3,tcp_speed_4,tcp_speed_5,tcp_speed_6\n";
  ros::Subscriber sub = nh_demo_get.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::spin();
  return 0;  		  	  		
}
