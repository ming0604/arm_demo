// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "tm_msgs/FeedbackState.h"

// std header
#include <sstream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>
#include <vector>
#include <cstdio>


std::fstream file;
std::fstream file_start;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"
// #include <Eigen/Dense>
#define pi 3.1415926
double joint[6];
// int joint_num = 0;
// using Eigen::VectorXd;
std::vector<std::vector<float>> read_csv(std::string filename){
    // string fname;
    std::vector<std::vector<float>> content;
    std::vector<float> row;
    std::string line, word;
    std::fstream file (filename, std::ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            row.clear();
 
            std::stringstream str(line);
 
            while(getline(str, word, ','))
                row.push_back(stof(word));
            content.push_back(row);
        }
    }
    else{
        std::cout<<"Could not open the file\n";
    }
    return content;

}
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
  // PID_file.open("/home/sheng/Documents/python_scripts/pyqt/read_data/path_sin.csv");
  // std::string myline;

  // float PID_para[16];
  
  // float amplitude[6];

  std::vector<std::vector<float>> data;

  int count = 0;
  float speed_saturation = 20.0;
  float init_joint[6];
  float goal_joint[6];
  // data = read_csv("/home/sheng/Documents/python_scripts/pyqt/read_data/path_sin.csv");
  data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/joint_smooth.csv");
  
  std::string cmd_init = "PTP(\"JPP\",";

  for (int i = 0; i < 6; i++){
    
    joint[i] = data[0][i];
    init_joint[i] = data[0][i];
    goal_joint[i] = data.back()[i];
    cmd_init += std::to_string(data[0][i]) + ",";
  }
  
  cmd_init += "80,100,0,false)";
  std::cout << "data size: " << data.size() << ", " << data[0].size() << std::endl;
  std::cout << "cmd_init: "<< cmd_init << std::endl;
  std::cout << "goal: ";
  for (int i = 0; i < 5; i++){
    std::cout << goal_joint[i] << ",";
  } 
  std::cout << goal_joint[5] << std::endl;
  std::string cmd_start = "ContinueVJog()";
  // std::string cmd_set_po = "SetContinueVJog(10,0,0,0,0,0)";
  // std::string cmd_set_ne = "SetContinueVJog(-10,0,0,0,0,0)";
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

  float sampling_rate = 100; // Hz

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
  file_start.open("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt", std::ios::out | std::ios::trunc);


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



  float kp = 5;
  float Ts =  1.0 / sampling_rate;
  count = 0;

  // float integral[6] ={0,0,0,0,0,0};
  // float deriva[6]   ={0,0,0,0,0,0};
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

  
  
  
  file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/data_save/PID_step_state_multi_file.csv", std::ios::out | std::ios::trunc);
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
      if (count >= data.size()){
        desired[i] = goal_joint[i];
        remove("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt");

      }
      else{
        desired[i] = data[count][i];
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
    // for (int i = 0; i < 6; i++){
    //   lower_bound = std::min(init_joint[i],goal_joint[i]) - 10.0;
    //   upper_bound = std::max(init_joint[i],goal_joint[i]) + 10.0;
    //   if ((joint[i] > upper_bound) || (joint[i] < lower_bound)){
    //     exceed_joint_bound++;
    //   }
    // }
    
        
    
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
          // integral[i] += error[i] * Ts;
          // deriva[i] = (error[i] - old_error[i]) / Ts;
          // command[i] = kp*error[i] + ki*integral[i] + kd*deriva[i];
          command[i] = kp*error[i];
          if (command[i] > speed_saturation){
            command[i] = speed_saturation;
          }
          else if (command[i] < -speed_saturation){
            command[i] = -speed_saturation;
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
