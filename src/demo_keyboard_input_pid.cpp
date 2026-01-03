// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>
// TM Driver header
#include "tm_msgs/SendScript.h"


#include <thread>

#define pi 3.1415926
double joint_1=90;

float kp = 5;
float ti = 15;
float td = 0.1;

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

void modify_PID(void){
  char c;

  while(true)
  {

    c = std::cin.get();
    if (c == '\n'){
      continue;
    }
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "     kp   ki   kd  (big)|   kp   ki   kd  (small)" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "up   q    w    e            r    t    y" << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "down a    s    d            f    g    h" << std::endl;
    std::cout << "------------------------------------------------------\n\n" << std::endl;
    std::cout << "\ncommand: ";
    
    double big = 1;
    double small = 0.0001;
    // q
    if (c == 'q'){
      kp += big;
      std::cout << "kp: " << kp << std::endl;

    }

    // a
    else if (c == 'a'){
      kp -= big;
      std::cout << "kp: " << kp << std::endl;
    }

    // w
    else if (c == 'w'){
      ti += big;
      std::cout << "ti: " << ti << std::endl;
    }

    // s
    else if (c == 's'){
      ti -= big;
      std::cout << "ti: " << ti << std::endl;
    }

    // e
    else if (c == 'e'){
      td += big;
      std::cout << "td: " << td << std::endl;
    }
    // d
    else if (c == 'd'){
      td -= big;
      std::cout << "td: " << td << std::endl;
    }

    // r
    else if (c == 'r'){
      kp += small;
      std::cout << "kp: " << kp << std::endl;
    }
    // f
    else if (c == 'f'){
      kp -= small;
      std::cout << "kp" << kp << std::endl;
    }
    // t
    else if (c == 't'){
      ti += small;
      std::cout << "ti: " << ti << std::endl;
    }
    // g
    else if (c == 'g'){
      ti -= small;
      std::cout << "ti: " << ti << std::endl;
    }
    // y
    else if (c == 'y'){
      td += small;
      std::cout << "td: " << td << std::endl;
    }
    // h
    else if (c == 'h'){
      td -= small;
      std::cout << "td: " << td << std::endl;
    }

    else if (c == 'x'){
      std::cout << "break" << std::endl;
      break;
    }
      // cout << "得到字元[" << c << "]  ASCII編號 [" << (int)c << "]" << endl;
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
  
  //Request	


  std::cout << joint_1 << std::endl;
  std::thread t1(modify_PID);
  
  while (ros::ok()){
      ros::spinOnce();
      // std::cout << joint_1 << std::endl;
      rate.sleep();

  }
  t1.join();

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
