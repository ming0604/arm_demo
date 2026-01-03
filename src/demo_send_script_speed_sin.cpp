// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include <fstream>
std::fstream file;
#define pi 3.1415926
double joint_1=90;



void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6)
  {
    joint_1 = ::atof(std::to_string(msg->joint_pos[0]).c_str())/3.1415926*180 ;
    // std::cout << joint_1/3.1415926*180 << std::endl;
    // ROS_INFO_STREAM("FeedbackState: joint pos = (" << 
    //             msg->joint_pos[0] << ", " << 
    //             msg->joint_pos[1] << ", " << 
    //             msg->joint_pos[2] << ", " <<
    //             msg->joint_pos[3] << ", " << 
    //             msg->joint_pos[4] << ", " << 
    //             msg->joint_pos[5] << ")"); 
  }
  else
  {
    ROS_ERROR_STREAM("Error FeedbackState callback"); 
  }
}

int main(int argc, char **argv)
{   
  std::string cmd_init = "PTP(\"JPP\",90,-30,96,57,85,70,80,100,0,false)"; 
  std::string cmd_start = "ContinueVJog()";
  std::string cmd_set_po = "SetContinueVJog(10,0,0,0,0,0)";
  std::string cmd_set_ne = "SetContinueVJog(-10,0,0,0,0,0)";
  std::string cmd_stop = "StopContinueVmode()";
  std::string front_s, back_s, result;
  double y;
  front_s = "SetContinueVJog(";
  back_s = ",0,0,0,0,0)";

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  ros::Rate rate(50);
  file.open("open_state.csv", std::ios::out | std::ios::trunc);
  file << "count,joint[0],desired,error\n";
  
  //Request	
  srv.request.id = "demo";
  srv.request.script = cmd_init;
  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Init joint => Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }




  srv.request.id = "demo";
  srv.request.script = cmd_start;
  // start
  for (int i = 11; i > 0; i--){
    ROS_INFO_STREAM("Countdown " << i << " sec");
    ros::Duration(1).sleep();
  }
  // ros::Duration(7).sleep();
  // ROS_INFO_STREAM("3");
  // ros::Duration(1).sleep();
  // ROS_INFO_STREAM("2");
  // ros::Duration(1).sleep();
  // ROS_INFO_STREAM("1");
  // ros::Duration(1).sleep();
  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Start Velocity Mode => Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }
//   set speed
  // srv.request.id = "demo";
  // srv.request.script = cmd_set_po;
  // if (client.call(srv))                             
  // {
  //   if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
  //   else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("Error send script to robot");
  //   return 1;
  // }

  
  int count=0;
  double desir;
  double error;
  double total_error;
  std::cout << joint_1 << std::endl;
  while (ros::ok()){
      ros::spinOnce();
      if (count == 1000){
          std::cout << joint_1 << std::endl;
      }
      // std::cout << joint_1 << std::endl;
      if (count == 600){
        count = 0;
      }
      count++;

      if (joint_1 > 130 || joint_1 < 50){
          srv.request.id = "demo";
          srv.request.script = cmd_stop;
          client.call(srv);
          break;
      }

      else{
          srv.request.id = "demo";
          desir = 15*sin(pi*count/300)+90;
          y = 5 * pi * cos(pi*count/300);
          result = front_s + std::to_string(y) + back_s;

          error = desir - joint_1;
          total_error += std::abs(error);
          file << std::to_string(count) + "," + std::to_string(joint_1) + "," + std::to_string(desir) + "\n";

          // std::cout << result << std::endl;
          srv.request.script = result;
          
          client.call(srv);
      }
      // if (joint_1 > 105){
         
      //     srv.request.id = "demo";
      //     srv.request.script = cmd_set_ne;
      //     client.call(srv);
      // }
      // else if (joint_1 < 75){
      //     srv.request.id = "demo";
      //     srv.request.script = cmd_set_po;
      //     client.call(srv);
      // }


      // ros::Duration(0.1).sleep();
      rate.sleep();

  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
