// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <chrono>


std::fstream file;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"
// #include <Eigen/Dense>
#define pi 3.1415926
double joint[6];
// int joint_num = 0;
// using Eigen::VectorXd;

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6)
  {
    for (int i = 0; i < 6; i++){
      joint[i] = std::atof(std::to_string(msg->joint_pos[i]).c_str())/3.1415926*180 ;
    }
    // joint_1 = std::atof(std::to_string(msg->joint_pos[joint_num]).c_str())/3.1415926*180 ;
  }
  else
  {
    ROS_ERROR_STREAM("Error FeedbackState callback"); 
  }
}

int main(int argc, char **argv)
{   
  PID_file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/PID_parameter_multi.csv");
  std::string myline;

  float PID_para[16];
  float init_joint[6];
  float goal_joint[6];
  float amplitude[6];

  int file_length = 16;
  // 0: P, 1: I, 2: D
  // 3: reach time
  // 4-9: init joint1-6
  // 10-15: goal joint1-6
  
  int count = 0;
  if ( PID_file.is_open() ) {
    while ( std::getline(PID_file, myline) ) {
      // std::cout << myline << '\n';
      PID_para[count] = std::stof(myline);
      std::cout << std::stof(myline) << std::endl;
      count += 1;
    }
  }
  
  

  // std::string cmd_init_1 = "PTP(\"JPP\","; 
  // std::string cmd_init_2 = std::to_string(PID_para[3]); 
  // std::string cmd_init_3 = ",-30,96,57,85,70,80,100,0,false)"; 
  std::string cmd_init = "PTP(\"JPP\",";
  // std::string joint[6];
  std::cout << "pid parameter: " << std::endl;
  for (int i = 0; i < file_length; i++){
    std::cout << PID_para[i] << std::endl;
  }
  std::cout << "====================================" << std::endl;
  for (int i = 0; i < 6; i++){
    
    cmd_init += std::to_string(PID_para[4+i]) + ",";
    joint[i] = PID_para[4+i];
    init_joint[i] = PID_para[4+i];
    goal_joint[i] = PID_para[10+i];
    amplitude[i] = (goal_joint[i] - init_joint[i]) / 2.0;

  }
  
  cmd_init += "80,100,0,false)";

  std::cout << "cmd_init: "<< cmd_init << std::endl;
  // std::string cmd_init = "PTP(\"JPP\"," + std::to_string(PID_para[3]) + ",-30,96,57,85,70,80,100,0,false)";
  // std::string cmd_goal = "PTP(\"JPP\"," + std::to_string(PID_para[4]) + ",-30,96,57,85,70,80,100,0,false)";
  // std::string cmd_goal = "PTP(\"JPP\",90,-30,96,57,85,70,80,100,0,false)"; 
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

  float sampling_rate = 50; // Hz

  ros::Rate rate(sampling_rate);
  
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
  for (int i = 20; i > 0; i--){
    ROS_INFO_STREAM("Countdown " << i << " sec");
    ros::Duration(1).sleep();
  }

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



  float kp = PID_para[0];
  float ki = PID_para[1];
  float kd = PID_para[2];
  float reach_time = PID_para[3];
  float T = reach_time*2;
  float Ts =  1.0 / sampling_rate;
  count = 0;

  float integral[6] ={0,0,0,0,0,0};
  float deriva[6]   ={0,0,0,0,0,0};
  float error[6]    ={0,0,0,0,0,0};
  float old_error[6]={0,0,0,0,0,0};
  float desired[6]  ={0,0,0,0,0,0};
  float command[6]  ={0,0,0,0,0,0};
  float lower_bound;
  float upper_bound;
  int exceed_joint_bound;
  
  std::cout << "=================joint_sensor_init=================" << std::endl;
  for (int i = 0; i < 6; i++){
    std::cout << "joint " << i << ": " << joint[i] << std::endl;
  }

  
  
  
  file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/data_save/PID_step_state_multi.csv", std::ios::out | std::ios::trunc);
  file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5]\n";
  
  
  
  // time_t start, end; 
  
  // time(&start); 
  //  clock_t start, end;
  //  start = clock();
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  double time_taken;
  
  while (ros::ok()){
    ros::spinOnce();
    std::cout << "Desired ==> ";
    for (int i = 0; i < 6; i++){
      if (count > (T/(2.0*Ts))){
        desired[i] = goal_joint[i];
      }
      else{
        desired[i] = amplitude[i]*sin(2.0*M_PI/T*(count*Ts)-M_PI_2) + amplitude[i] + init_joint[i];
      }
      std::cout << i << ": " << std::fixed << desired[i] << std::setprecision(5) << ", ";
    }
    std::cout << std::endl;
    
    
    end = std::chrono::steady_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
    
    if (count == 2147483000){
      count = 0;
    }
    count++;

    exceed_joint_bound = 0;
    for (int i = 0; i < 6; i++){
      lower_bound = std::min(init_joint[i],goal_joint[i]) - 10.0;
      upper_bound = std::max(init_joint[i],goal_joint[i]) + 10.0;
      if ((joint[i] > upper_bound) || (joint[i] < lower_bound)){
        exceed_joint_bound++;
      }
    }
    
        
    
    if (exceed_joint_bound > 0){
      srv.request.id = "demo";
      srv.request.script = cmd_stop;
      client.call(srv);
      std::cout << "stop pid program" << std::endl;
      break;
    }

    else{
        
        srv.request.id = "demo";
        for (int i = 0; i < 6; i++){
          error[i] = desired[i] - joint[i];
          integral[i] += error[i] * Ts;
          deriva[i] = (error[i] - old_error[i]) / Ts;
          command[i] = kp*error[i] + ki*integral[i] + kd*deriva[i];
          if (command[i] > 10){
            command[i] = 10;
          }
          else if (command[i] < -10){
            command[i] = -10;
          }
          old_error[i] = error[i];
        }

        result = front_s;
        for (int i = 0; i < 6; i++){
          result += std::to_string(command[i]);
          
          if (i==5){
            result += ")";
          }
          else{
            result += ",";
          }
        }
        std::cout << result << std::endl;
        
        file << std::to_string(count);
        for (int i = 0; i < 6; i++){
          file << "," + std::to_string(joint[i]);
        }
        for (int i = 0; i < 6; i++){
          file << "," + std::to_string(desired[i]);
        }
        for (int i = 0; i < 6; i++){
          file << "," + std::to_string(command[i]);
        }
        file << "\n";
        
        
        srv.request.script = result;
        
        client.call(srv);
      }
      rate.sleep();

  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}
