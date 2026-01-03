// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>
#include <fstream>
std::fstream file;
// TM Driver header
#include "tm_msgs/SendScript.h"
#define pi 3.1415926
double joint_1=89.5;


void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6)
  {
    joint_1 = ::atof(std::to_string(msg->joint_pos[0]).c_str())/3.1415926*180 ;
  }
  else
  {
    ROS_ERROR_STREAM("Error FeedbackState callback"); 
  }
}

int main(int argc, char **argv)
{   
  std::string cmd_init = "PTP(\"JPP\",89,-30,96,57,85,70,80,100,0,false)"; 
  std::string cmd_goal = "PTP(\"JPP\",90,-30,96,57,85,70,80,100,0,false)"; 
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
  // srv.request.script = cmd_start;
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
  // if (client.call(srv))                             
  // {
  //   if (srv.response.ok) ROS_INFO_STREAM("Start Velocity Mode => Sent script to robot");
  //   else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("Error send script to robot");
  //   return 1;
  // }
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
  // float kp = 7.2;
  // float ki = 14.1332;
  // float kd = 0.9169875;

  // float kp = 5;
  // float ti = 15;
  // float td = 0.15;

  // float kp = 5;
  // float ti = 15;
  // float td = 0.15;

  float kp = 1;
  float ti = 3.09;
  float td = 0.1;

  float integral = 0;
  float deriva; 
  float sampling_time = 0.02;
  float desir = 90; 
  float error;
  float old_error = 90 - 89.5;
  std::cout << joint_1 << std::endl;
  file.open("PID_step_state.csv", std::ios::out | std::ios::trunc);
  file << "count,joint[0],desired,error,kp,ti,td\n";
  while (ros::ok()){
      ros::spinOnce();
      if (count == 0){
        srv.request.id = "demo";
        srv.request.script = cmd_goal;
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
        
      }
      file << std::to_string(count) + "," + std::to_string(joint_1) + "," + std::to_string(desir) + "\n";
      count++;
      // if (count == 1000){
      //     std::cout << joint_1 << std::endl;
      // }
      // // std::cout << joint_1 << std::endl;
      // if (count == 600){
      //   count = 0;
      // }
      // count++;

      // if (joint_1 > 110 || joint_1 < 70){
      //     srv.request.id = "demo";
      //     srv.request.script = cmd_stop;
      //     client.call(srv);
      //     break;
      // }

      // else{
          
      //     srv.request.id = "demo";
      //     error = desir - joint_1;

      //     integral += error * sampling_time;
      //     deriva = (error - old_error) / sampling_time;
      //     y = kp * (error + ti  * integral + td * deriva);
      //     old_error = error;
      //     std::cout << "before: "  << std::setw(10) << y  ;
      //     if (y > 10){
      //       y = 10;
      //     }
      //     else if (y < -10){
      //       y = -10;
      //     }
      //     std::cout << ", after: " << std::setw(10) << y ;
      //     std::cout << ", error: " << std::setw(10) << error << std::endl;
      //     // std::cout << ", p" << std::setw(10) << kp * error;
      //     // std::cout << ", i" << std::setw(10) << ki * integral;
      //     // std::cout << ", d" << std::setw(10) << kd * deriva << std::endl;
            
      //     result = front_s + std::to_string(y) + back_s;
      //     file << std::to_string(count) + "," + std::to_string(joint_1) + "," + std::to_string(desir) + "," + std::to_string(error) + "\n";

      //     // std::cout << result << std::endl;
      //     srv.request.script = result;
          
      //     client.call(srv);
      // }
      rate.sleep();

  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
