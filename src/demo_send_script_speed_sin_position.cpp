// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
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
  std::string cmd_init = "PTP(\"JPP\",90,-30,96,57,85,70,100,0,100,false)"; 
  std::string front_s, back_s, result;
  double y;
  front_s = "PTP(\"JPP\",";
  back_s = ",-30,96,57,85,70,100,0,100,false)";

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  ros::Rate rate(100);
  
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
  // start
  for (int i = 11; i > 0; i--){
    ROS_INFO_STREAM("Countdown " << i << " sec");
    ros::Duration(1).sleep();
  }
  
  long count=0;
  std::cout << joint_1 << std::endl;

  while (ros::ok()){
      ros::spinOnce();
      // std::cout << joint_1 << std::endl;
      if (count == 600){
        count = 0;
      }
      count++;

      if (joint_1 > 110 || joint_1 < 70){
          srv.request.id = "demo";
          srv.request.script = cmd_init;
          client.call(srv);
          std::cout << "position command out of range." << std::endl;
          break;
      }

      else{
          srv.request.id = "demo";
          
          y = 15*sin(pi*count/300)+90;
          if (y > 110 || y < 70){
            std::cout << "position command out of range." << std::endl;
            break;
          }
          result = front_s + std::to_string(y) + back_s;
          // std::cout << result << std::endl;
          srv.request.script = result;
          
          client.call(srv);
      }
      rate.sleep();

  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
