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
#include <iomanip> // 為了 std::setprecision
#include <iostream> // 引入 iostream

std::fstream file;
std::fstream file_start;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"

#define pi 3.1415926

// 全域變數宣告
double joint[6];
double actual_joint_vel[6]; // 用來存實際角速度 (deg/s)

std::vector<std::vector<float>> read_csv(std::string filename){
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
        std::cout<<"Could not open the file: " << filename << "\n";
    }
    return content;
}

// Callback 函式
void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  // 讀取位置
  if(msg->joint_pos.size() == 6)
  {
    for (int i = 0; i < 6; i++){
      joint[i] = std::atof(std::to_string(msg->joint_pos[i]).c_str())/3.1415926*180 ;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error FeedbackState callback (Pos)"); 
  }

  // 讀取速度
  if(msg->joint_vel.size() == 6)
  {
    for (int i = 0; i < 6; i++){
       actual_joint_vel[i] = msg->joint_vel[i] * 180.0 / 3.1415926;
    }
  }
  else
  {
     for (int i = 0; i < 6; i++) actual_joint_vel[i] = 0.0;
  }
}

int main(int argc, char **argv)
{   
  // ROS node initialization
  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::NodeHandle nh_private("~"); // for getting private parameters
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;  

  std::string output_file_path;
  std::string csv_path;
  float kp;
  // float kd = 0.001;

  // get parameters from launch file or command line
  nh_private.param<std::string>("traj_csv_path", csv_path, "/Default/path");
  nh_private.param<std::string>("output_file_path", output_file_path, "/Default/path");
  nh_private.param<float>("kp_gain", kp , 5.0);
  std::cout << "CSV Path: " << csv_path << std::endl;
  std::cout << "Output File Path: " << output_file_path << std::endl;
  std::cout << "Kp Gain: " << kp << std::endl;
  std::vector<std::vector<float>> data;

  int count = 0;
  float speed_saturation = 20.0;
  float position_tolerance = 2.0; 

  float goal_joint[6];
  
  // Read the desired joint positions from CSV file
  data = read_csv(csv_path);

  if (data.empty()) {
      ROS_ERROR("CSV data is empty or file not found!");
      return -1;
  }

  for (int i = 0; i < 6; i++){
    joint[i] = data[0][i];
    goal_joint[i] = data.back()[i];
  }
  
  std::cout << "data size: " << data.size() << ", " << data[0].size() << std::endl;
  std::cout << "goal: ";
  for (int i = 0; i < 5; i++){
    std::cout << goal_joint[i] << ",";
  } 
  std::cout << goal_joint[5] << std::endl;
  
  // Set the command strings for starting and stopping velocity mode
  std::string cmd_start = "ContinueVJog()";
  std::string cmd_stop = "StopContinueVmode()";
  std::string front_s, back_s, result;
  front_s = "SetContinueVJog(";
  back_s = ",0,0,0,0,0)";



  float sampling_rate = 100; // Hz

  ros::Rate rate(sampling_rate);

  // Send command to start velocity mode
  srv.request.id = "demo";
  srv.request.script = cmd_start;
  
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


  count = 0;

  // float integral[6] ={0,0,0,0,0,0};
  // float deriva[6]   ={0,0,0,0,0,0};
  float error[6]    ={0,0,0,0,0,0};
  float old_error[6]={0,0,0,0,0,0};
  float desired[6]  ={0,0,0,0,0,0};
  float command[6]  ={0,0,0,0,0,0};
  int exceed_joint_bound = 0;
  
  std::cout << "=================joint_sensor_init=================" << std::endl;
  for (int i = 0; i < 6; i++){
    std::cout << "joint " << i << ": " << joint[i] << std::endl;
  }
  
  file.open(output_file_path, std::ios::out | std::ios::trunc);
  
  // CSV 標頭
  file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5],act_v[0],act_v[1],act_v[2],act_v[3],act_v[4],act_v[5]\n";
  
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  ROS_INFO_STREAM("Starting Control Loop...");
  // ========================== PID Control Loop ==========================
  while (ros::ok()){
    ros::spinOnce(); 
    
    bool path_finished = false; 

    // 顯示 desired joint (為了版面乾淨，這裡可以註解掉，或保留)
    // std::cout << "Desired ==> ... " << std::endl; 
    
    end = std::chrono::steady_clock::now();
    start = std::chrono::steady_clock::now();
    

    if (exceed_joint_bound > 0){
      srv.request.id = "demo";
      srv.request.script = cmd_stop;
      client.call(srv);
      std::cout << "Exceed bound: stop pid program" << std::endl;
      break;
    }
    else{
        // 讀取 CSV 目標點
        if (count >= data.size()){
            for(int i=0; i<6; i++) desired[i] = goal_joint[i];
            path_finished = true; 
            remove("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt");
        }
        else{
            for(int i=0; i<6; i++) desired[i] = data[count][i];
        }

        // 計算 Error 與 Command
        for (int i = 0; i < 6; i++){
          error[i] = desired[i] - joint[i];
          command[i] = kp * error[i]; 
          // integral[i] += error[i] * Ts;
          // deriva[i] = (error[i] - old_error[i]) / Ts;
          // command[i] = kp*error[i] + ki*integral[i] + kd*deriva[i];
          // command[i] = (kp * error[i]) + (kd * error_derivative);
          if (command[i] > speed_saturation) 
          {
            command[i] = speed_saturation;
          }
          else if (command[i] < -speed_saturation) 
          {
            command[i] = -speed_saturation;
          }
          old_error[i] = error[i];
        }

        // 檢查是否到達
        if (path_finished) {
            bool reached = true;
            for(int i=0; i<6; i++) {
                if (std::abs(error[i]) > position_tolerance) {
                    reached = false;
                    break;
                }
            }

            if (reached) {
                ROS_INFO_STREAM("Target Reached within tolerance (" << position_tolerance << " deg).");
                
                // 1. 停止速度模式
                srv.request.id = "demo";
                srv.request.script = cmd_stop;
                client.call(srv);
                ROS_INFO_STREAM("Velocity Mode Stopped.");
                
                // 2. 關閉檔案
                file.close();
                if (file_start.is_open()) file_start.close();

                // 3. 跳出 Control Loop (重要！)
                break; 
            }
        }

        // 發送速度指令
        result = front_s;
        for (int i = 0; i < 6; i++)
        {
          result += std::to_string(command[i]);
          if (i==5) 
          {
            result += ")";
          }
          else 
          {
            result += ",";
          }
        }
        
        file << std::to_string(count);
        for (int i = 0; i < 6; i++) file << "," + std::to_string(joint[i]);
        for (int i = 0; i < 6; i++) file << "," + std::to_string(desired[i]);
        for (int i = 0; i < 6; i++) file << "," + std::to_string(command[i]);
        for (int i = 0; i < 6; i++) file << "," + std::to_string(actual_joint_vel[i]);
        file << "\n";
        
        srv.request.id = "demo";
        srv.request.script = result;
        client.call(srv);

        if (count == 2147483000){
          count = 0;
        }
        count++;
      }
      
      rate.sleep();
  }
  // ========================== End of Loop ==========================

  ROS_INFO_STREAM("Program Finished.");
  return 0;
}
