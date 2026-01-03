// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>
#include <thread>
// TM Driver header
#include "tm_msgs/SendScript.h"
#define pi 3.1415926
double joint_1=90;
float kp = 5;
float ti = 1/15;
float td = 0.15;
int initial = 0;
int stop_program = 0;
#include <fstream>
std::fstream file; 
// int count=0;

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
    double small = 0.01;
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
      stop_program = 1;
      break;
    }
    initial = 1;
    
      // cout << "得到字元[" << c << "]  ASCII編號 [" << (int)c << "]" << endl;
  }
}

int main(int argc, char **argv)
{   
  std::string cmd_init = "PTP(\"JPP\",90,-30,96,57,85,70,80,50,0,false)"; 
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
  std::thread t1(modify_PID);
  file.open("PID_state.csv", std::ios::out | std::ios::trunc);
  file << "count,joint[0],desired,error,kp,ti,td\n";
  
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
  for (int i = 15; i > 0; i--){
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
  // float kp = 7.2;
  // float ki = 14.1332;
  // float kd = 0.9169875;

  // float kp = 5;
  // float ti = 15;
  // float td = 0.15;


  float integral = 0;
  float deriva; 
  float sampling_time = 0.02;
  float desir = 90; 
  float error;
  float old_error;
  int count=0;
  double total_error = 0;
  double average_error = 0;
  
  std::cout << joint_1 << std::endl;
  while (ros::ok()){
      ros::spinOnce();
      if (stop_program){
        srv.request.id = "demo";
        srv.request.script = cmd_stop;
        client.call(srv);
        break;
      }
      
      // std::cout << joint_1 << std::endl;
      if (count == 1800){
        count = 1200;
      }

      if (initial){
        srv.request.script = cmd_stop;
        client.call(srv);

        srv.request.script = cmd_init;
        client.call(srv);
        
        
        for (int i = 15; i > 0; i--){
          ROS_INFO_STREAM("Countdown " << i << " sec");
          ros::Duration(1).sleep();
        }
        srv.request.script = cmd_start;
        client.call(srv);

        initial = 0;
        count = 0;
        total_error = 0;
        integral = 0;
      }
      

      if (joint_1 > 120 || joint_1 < 60){
          srv.request.id = "demo";
          srv.request.script = cmd_stop;
          client.call(srv);
          break;
      }

      else{
        desir = 15*sin(pi*count/300)+90;
        error = desir - joint_1;
        total_error += std::abs(error);
        file << std::to_string(count) + "," + std::to_string(joint_1) + "," + std::to_string(desir) + "," + std::to_string(error) + "," + std::to_string(kp) + "," + std::to_string(ti) + "," + std::to_string(td) + "\n";

        
        if (count == 0){
          count++;
          old_error = error;
          continue;
        }
        if (count == 600){
          average_error = total_error / 301;
          std::cout << "================ Result ===============" << std::endl;
          std::cout << "kp: " << kp << ", ti: " << ti << ", td: " << td << std::endl; 
          std::cout << "average_error: " << average_error << std::endl;
          std::cout << "================  End  ===============" << std::endl;
        }
        
      
        srv.request.id = "demo";
        

        integral += error * sampling_time;
        deriva = (error - old_error) / sampling_time;
        y = kp * (error + ti  * integral + td * deriva);
        old_error = error;
        if (y > 10){
          y = 10;
        }
        else if (y < -10){
          y = -10;
        }
        result = front_s + std::to_string(y) + back_s;
        // std::cout << result << std::endl;
        srv.request.script = result;
        
        client.call(srv);
      }
      count++;
      rate.sleep();

  }
  t1.join();

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
