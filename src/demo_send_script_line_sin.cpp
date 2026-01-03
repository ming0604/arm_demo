// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include <Eigen/Dense>


using Eigen::VectorXd;
 
VectorXd v, x, y;

int main(int argc, char **argv)
{    
  std::string s="";
  std::string front="Line(\"CAP\",";
  std::string back=",380,180,0,90,800,1,100,false)\n";
  int num_sp = 1000;
  v.setLinSpaced(num_sp, 0, 2*M_PI*100);
  x = v / 100;
  y = x.array().sin()*150;
  for (int i = 0; i < num_sp; i++){
    s += front + std::to_string(430 + y[i]) + "," + std::to_string(v[i]  -330) + back;
  }
  std::string cmd = s;

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;
    
  //Request	
  srv.request.id = "demo";
  srv.request.script = cmd;

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
