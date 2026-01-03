
/*// ROS headers
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
  // data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/joint_smooth.csv");
  data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/left8cm.csv");
  
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
  for (int i = 10; i > 0; i--){
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
*/

/*
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

std::fstream file;
std::fstream file_start;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"

#define pi 3.1415926
double joint[6];

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

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6)
  {
    for (int i = 0; i < 6; i++){
      joint[i] = std::atof(std::to_string(msg->joint_pos[i]).c_str())/3.1415926*180 ;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error FeedbackState callback"); 
  }
}

int main(int argc, char **argv)
{   
  std::vector<std::vector<float>> data;

  int count = 0;
  float speed_saturation = 20.0;
  
  // ==========================================
  // [設定] 到達目標的容許誤差 (單位: 度)
  // 如果覺得太難停下來，可以把這個數值改大一點 (例如 2.0)
  float position_tolerance = 2.0; 
  // ==========================================

  float init_joint[6];
  float goal_joint[6];
  
  // 請確認路徑是否正確
  data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/1220x12cm.csv");
  
  if (data.empty()) {
      ROS_ERROR("CSV data is empty or file not found!");
      return -1;
  }

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
  std::string cmd_stop = "StopContinueVmode()";
  std::string front_s, back_s, result;
  front_s = "SetContinueVJog(";
  back_s = ",0,0,0,0,0)";

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  float sampling_rate = 100; // Hz

  ros::Rate rate(sampling_rate);
  
  // 1. 回到初始點
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

  // 2. 啟動速度模式
  srv.request.id = "demo";
  srv.request.script = cmd_start;
  
  // 倒數計時
  for (int i = 3; i > 0; i--){
    ROS_INFO_STREAM("Countdown " << i << " sec");
    ros::Duration(1).sleep();
  }
  
  // 建立同步檔案 (若不需要可註解)
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
  count = 0;

  float error[6]    ={0,0,0,0,0,0};
  float old_error[6]={0,0,0,0,0,0};
  float desired[6]  ={0,0,0,0,0,0};
  float command[6]  ={0,0,0,0,0,0};
  int exceed_joint_bound = 0;
  
  std::cout << "=================joint_sensor_init=================" << std::endl;
  for (int i = 0; i < 6; i++){
    std::cout << "joint " << i << ": " << joint[i] << std::endl;
  }
  
  file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/data_save/PID_step_state_multi_file.csv", std::ios::out | std::ios::trunc);
  file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5]\n";
  
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  while (ros::ok()){
    ros::spinOnce();
    
    // 生成 Desired 軌跡
    bool path_finished = false; // 標記路徑是否已經讀取完畢

    std::cout << "Desired ==> ";
    for (int i = 0; i < 6; i++){
      if (count >= data.size()){
        desired[i] = goal_joint[i];
        path_finished = true; 
        remove("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt");
      }
      else{
        desired[i] = data[count][i];
      }
      std::cout << i << ": " << std::fixed << desired[i] << std::setprecision(5) << ", ";
    }
    std::cout << std::endl;
    
    // 時間計算
    end = std::chrono::steady_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    start = std::chrono::steady_clock::now();
    
    // 防止 count 溢位 (雖然不太可能發生，但保留原邏輯)
    if (count == 2147483000){
      count = 0;
    }
    count++;

    // 安全停止邏輯 (原程式的 exceed_joint_bound 邏輯)
    if (exceed_joint_bound > 0){
      srv.request.id = "demo";
      srv.request.script = cmd_stop;
      client.call(srv);
      std::cout << "Exceed bound: stop pid program" << std::endl;
      break;
    }
    else{
        // PID 計算
        for (int i = 0; i < 6; i++){
          error[i] = desired[i] - joint[i];
          
          command[i] = kp * error[i]; // P Control

          // 速度飽和限制
          if (command[i] > speed_saturation){
            command[i] = speed_saturation;
          }
          else if (command[i] < -speed_saturation){
            command[i] = -speed_saturation;
          }
          old_error[i] = error[i];
        }

        // ========================================================
        // [新增功能] 到達目標點後的停止機制
        // ========================================================
        if (path_finished) {
            bool reached = true;
            for(int i=0; i<6; i++) {
                // 檢查所有關節的誤差絕對值是否都在容許範圍內
                if (std::abs(error[i]) > position_tolerance) {
                    reached = false;
                    break;
                }
            }

            if (reached) {
                ROS_INFO_STREAM("Target Reached within tolerance (" << position_tolerance << " deg). Stopping robot.");
                
                // 1. 發送停止指令
                srv.request.id = "demo";
                srv.request.script = cmd_stop;
                client.call(srv);
                
                // 2. 寫入最後一筆資料並關閉檔案
                file.close();
                if (file_start.is_open()) file_start.close();

                // 3. 跳出迴圈，結束程式
                break;
            }
        }
        // ========================================================

        // 組合指令字串
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
        // std::cout << result << std::endl;
        
        // 寫入 CSV
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
        
        // 發送指令
        srv.request.id = "demo";
        srv.request.script = result;
        client.call(srv);
      }
      
      rate.sleep();
  }

  ROS_INFO_STREAM("Program Finished.");
  return 0;
}*/
  
// // it can stop when reach the target position within tolerance
// // ROS headers
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include "tm_msgs/FeedbackState.h"

// // std header
// #include <sstream>
// #include <cstdlib>
// #include <fstream>
// #include <string>
// #include <cmath>
// #include <chrono>
// #include <vector>
// #include <cstdio>
// #include <iomanip> // 為了 std::setprecision

// std::fstream file;
// std::fstream file_start;
// std::fstream PID_file;
// // TM Driver header
// #include "tm_msgs/SendScript.h"

// #define pi 3.1415926

// // [修改 1] 全域變數宣告
// double joint[6];
// double actual_joint_vel[6]; // [新增] 用來存實際角速度 (deg/s)

// std::vector<std::vector<float>> read_csv(std::string filename){
//     std::vector<std::vector<float>> content;
//     std::vector<float> row;
//     std::string line, word;
//     std::fstream file (filename, std::ios::in);
//     if(file.is_open())
//     {
//         while(getline(file, line))
//         {
//             row.clear();
//             std::stringstream str(line);
//             while(getline(str, word, ','))
//                 row.push_back(stof(word));
//             content.push_back(row);
//         }
//     }
//     else{
//         std::cout<<"Could not open the file: " << filename << "\n";
//     }
//     return content;
// }

// // [修改 2] 修改 Callback 函式，讀取實際速度
// void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
// {
//   // 讀取位置
//   if(msg->joint_pos.size() == 6)
//   {
//     for (int i = 0; i < 6; i++){
//       joint[i] = std::atof(std::to_string(msg->joint_pos[i]).c_str())/3.1415926*180 ;
//     }
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Error FeedbackState callback (Pos)"); 
//   }

//   // [新增] 讀取速度 (注意: msg->joint_vel 單位通常是 rad/s，需轉成 deg/s)
//   if(msg->joint_vel.size() == 6)
//   {
//     for (int i = 0; i < 6; i++){
//        // 轉換公式: rad/s * 180 / pi = deg/s
//        actual_joint_vel[i] = msg->joint_vel[i] * 180.0 / 3.1415926;
//     }
//   }
//   else
//   {
//      // 防呆：如果沒有讀到速度，補 0
//      for (int i = 0; i < 6; i++) actual_joint_vel[i] = 0.0;
//   }
// }

// int main(int argc, char **argv)
// {   
//   std::vector<std::vector<float>> data;

//   int count = 0;
//   float speed_saturation = 20.0;
//   float position_tolerance = 2.0; 

//   float init_joint[6];
//   float goal_joint[6];
  
//   // 請確認路徑是否正確
//   data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/1222z12cm.csv");
  
//   if (data.empty()) {
//       ROS_ERROR("CSV data is empty or file not found!");
//       return -1;
//   }

//   std::string cmd_init = "PTP(\"JPP\",";

//   for (int i = 0; i < 6; i++){
//     joint[i] = data[0][i];
//     init_joint[i] = data[0][i];
//     goal_joint[i] = data.back()[i];
//     cmd_init += std::to_string(data[0][i]) + ",";
//   }
  
//   cmd_init += "80,100,0,false)";
//   std::cout << "data size: " << data.size() << ", " << data[0].size() << std::endl;
//   std::cout << "cmd_init: "<< cmd_init << std::endl;
//   std::cout << "goal: ";
//   for (int i = 0; i < 5; i++){
//     std::cout << goal_joint[i] << ",";
//   } 
//   std::cout << goal_joint[5] << std::endl;

//   std::string cmd_start = "ContinueVJog()";
//   std::string cmd_stop = "StopContinueVmode()";
//   std::string front_s, back_s, result;
//   front_s = "SetContinueVJog(";
//   back_s = ",0,0,0,0,0)";

//   ros::init(argc, argv, "demo_send_script");      
//   ros::NodeHandle nh_demo; 
//   ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
//   ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
//   tm_msgs::SendScript srv;

//   float sampling_rate = 100; // Hz

//   ros::Rate rate(sampling_rate);
  
//   // 1. 回到初始點
//   srv.request.id = "demo";
//   srv.request.script = cmd_init;
//   if (client.call(srv))                             
//   {
//     if (srv.response.ok) ROS_INFO_STREAM("Init joint => Sent script to robot");
//     else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Error send script to robot");
//     return 1;
//   }

//   // 2. 啟動速度模式
//   srv.request.id = "demo";
//   srv.request.script = cmd_start;
  
//   for (int i = 3; i > 0; i--){
//     ROS_INFO_STREAM("Countdown " << i << " sec");
//     ros::Duration(1).sleep();
//   }
  
//   file_start.open("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt", std::ios::out | std::ios::trunc);

//   if (client.call(srv))                             
//   {
//     if (srv.response.ok) ROS_INFO_STREAM("Start Velocity Mode => Sent script to robot");
//     else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Error send script to robot");
//     return 1;
//   }

//   float kp = 0.5;
//   count = 0;

//   float error[6]    ={0,0,0,0,0,0};
//   float old_error[6]={0,0,0,0,0,0};
//   float desired[6]  ={0,0,0,0,0,0};
//   float command[6]  ={0,0,0,0,0,0};
//   int exceed_joint_bound = 0;
  
//   std::cout << "=================joint_sensor_init=================" << std::endl;
//   for (int i = 0; i < 6; i++){
//     std::cout << "joint " << i << ": " << joint[i] << std::endl;
//   }
  
//   file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/data_save/PID_step_state_multi_file.csv", std::ios::out | std::ios::trunc);
  
//   // [修改 3] CSV 標頭增加 act_v 欄位
//   file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5],act_v[0],act_v[1],act_v[2],act_v[3],act_v[4],act_v[5]\n";
  
//   auto start = std::chrono::steady_clock::now();
//   auto end = std::chrono::steady_clock::now();

//   while (ros::ok()){
//     ros::spinOnce(); // 注意：這個會觸發 callback 更新 actual_joint_vel
    
//     bool path_finished = false; 

//     std::cout << "Desired ==> ";
//     for (int i = 0; i < 6; i++){
//       if (count >= data.size()){
//         desired[i] = goal_joint[i];
//         path_finished = true; 
//         remove("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt");
//       }
//       else{
//         desired[i] = data[count][i];
//       }
//       std::cout << i << ": " << std::fixed << desired[i] << std::setprecision(5) << ", ";
//     }
//     std::cout << std::endl;
    
//     end = std::chrono::steady_clock::now();
//     start = std::chrono::steady_clock::now();
    
//     if (count == 2147483000){
//       count = 0;
//     }
//     count++;

//     if (exceed_joint_bound > 0){
//       srv.request.id = "demo";
//       srv.request.script = cmd_stop;
//       client.call(srv);
//       std::cout << "Exceed bound: stop pid program" << std::endl;
//       break;
//     }
//     else{
//         for (int i = 0; i < 6; i++){
//           error[i] = desired[i] - joint[i];
//           command[i] = kp * error[i]; 

//           if (command[i] > speed_saturation){
//             command[i] = speed_saturation;
//           }
//           else if (command[i] < -speed_saturation){
//             command[i] = -speed_saturation;
//           }
//           old_error[i] = error[i];
//         }

//         if (path_finished) {
//             bool reached = true;
//             for(int i=0; i<6; i++) {
//                 if (std::abs(error[i]) > position_tolerance) {
//                     reached = false;
//                     break;
//                 }
//             }

//             if (reached) {
//                 ROS_INFO_STREAM("Target Reached within tolerance (" << position_tolerance << " deg). Stopping robot.");
//                 srv.request.id = "demo";
//                 srv.request.script = cmd_stop;
//                 client.call(srv);
//                 file.close();
//                 if (file_start.is_open()) file_start.close();
//                 break;
//             }
//         }

//         result = front_s;
//         for (int i = 0; i < 6; i++){
//           result += std::to_string(command[i]);
//           if (i==5) result += ")";
//           else result += ",";
//         }
        
//         // [修改 4] 寫入 CSV
//         file << std::to_string(count);
//         for (int i = 0; i < 6; i++) file << "," + std::to_string(joint[i]);
//         for (int i = 0; i < 6; i++) file << "," + std::to_string(desired[i]);
//         for (int i = 0; i < 6; i++) file << "," + std::to_string(command[i]);
        
//         // [新增] 寫入實際速度
//         for (int i = 0; i < 6; i++) file << "," + std::to_string(actual_joint_vel[i]);
        
//         file << "\n";
        
//         srv.request.id = "demo";
//         srv.request.script = result;
//         client.call(srv);
//       }
      
//       rate.sleep();
//   }

//   ROS_INFO_STREAM("Program Finished.");
//   return 0;
// }


////////

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
  std::vector<std::vector<float>> data;

  int count = 0;
  float speed_saturation = 20.0;
  float position_tolerance = 2.0; 

  float init_joint[6];
  float goal_joint[6];
  
  // 請確認路徑是否正確
  data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/x3.5cm.csv");
  
  if (data.empty()) {
      ROS_ERROR("CSV data is empty or file not found!");
      return -1;
  }

  // 建立回到初始點的 PTP 指令字串
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
  std::string cmd_stop = "StopContinueVmode()";
  std::string front_s, back_s, result;
  front_s = "SetContinueVJog(";
  back_s = ",0,0,0,0,0)";

  ros::init(argc, argv, "demo_send_script");      
  ros::NodeHandle nh_demo; 
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  float sampling_rate = 100; // Hz

  ros::Rate rate(sampling_rate);
  
  // 1. 先回到初始點 (程式啟動時)
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

  // 2. 啟動速度模式
  srv.request.id = "demo";
  srv.request.script = cmd_start;
  
  for (int i = 15; i > 0; i--){
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
  float kd = 0.001;
  count = 0;

  float error[6]    ={0,0,0,0,0,0};
  float old_error[6]={0,0,0,0,0,0};
  float desired[6]  ={0,0,0,0,0,0};
  float command[6]  ={0,0,0,0,0,0};
  int exceed_joint_bound = 0;
  
  std::cout << "=================joint_sensor_init=================" << std::endl;
  for (int i = 0; i < 6; i++){
    std::cout << "joint " << i << ": " << joint[i] << std::endl;
  }
  
  file.open("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/data_save/PID_step_state_multi_file.csv", std::ios::out | std::ios::trunc);
  
  // CSV 標頭
  file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5],act_v[0],act_v[1],act_v[2],act_v[3],act_v[4],act_v[5]\n";
  
  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  // ========================== PID Control Loop ==========================
  while (ros::ok()){
    ros::spinOnce(); 
    
    bool path_finished = false; 

    // 顯示 desired joint (為了版面乾淨，這裡可以註解掉，或保留)
    // std::cout << "Desired ==> ... " << std::endl; 
    
    end = std::chrono::steady_clock::now();
    start = std::chrono::steady_clock::now();
    
    if (count == 2147483000){
      count = 0;
    }
    count++;

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
          // command[i] = kp * error[i]; 
          float error_derivative = (error[i] - old_error[i]) * sampling_rate;
          command[i] = (kp * error[i]) + (kd * error_derivative);
          if (command[i] > speed_saturation) command[i] = speed_saturation;
          else if (command[i] < -speed_saturation) command[i] = -speed_saturation;
          
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
        for (int i = 0; i < 6; i++){
          result += std::to_string(command[i]);
          if (i==5) result += ")";
          else result += ",";
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
      }
      
      rate.sleep();
  }
  // ========================== End of Loop ==========================


  // ========================== 等待按鍵並回原點 ==========================
  // 確認 ROS 還活著 (避免使用者已經按 Ctrl+C 強制結束)
  if (ros::ok()) {
      std::cout << "\n";
      std::cout << "##################################################" << std::endl;
      std::cout << "     Task Finished. Robot stopped." << std::endl;
      std::cout << "     Press [Enter] key to return to Start Position..." << std::endl;
      std::cout << "##################################################" << std::endl;

      // 清空輸入緩衝區 (避免之前的殘留字元)
      // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
      // 簡單的等待 Enter (讀取一個字元)
      std::cin.get(); 

      ROS_INFO_STREAM("Key pressed! Sending command to return to Start Point...");

      // 發送 PTP 指令回到初始點
      srv.request.id = "demo";
      srv.request.script = cmd_init;
      
      if (client.call(srv)) {
          if (srv.response.ok) 
              ROS_INFO_STREAM("Return Script Sent successfully.");
          else 
              ROS_WARN_STREAM("Return Script sent but response not OK.");
      } else {
          ROS_ERROR_STREAM("Failed to send Return Script.");
      }
  }

  ROS_INFO_STREAM("Program Finished.");
  return 0;
}

// // =============================================================================
// //  ROS & Standard Headers
// // =============================================================================
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include "tm_msgs/FeedbackState.h"
// #include "tm_msgs/SendScript.h"

// #include <sstream>
// #include <cstdlib>
// #include <fstream>
// #include <string>
// #include <cmath>
// #include <vector>
// #include <cstdio>
// #include <iomanip>
// #include <iostream>

// // =============================================================================
// //  Eigen Headers (矩陣運算核心)
// // =============================================================================
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// // =============================================================================
// //  全域變數與常數定義
// // =============================================================================
// #define PI 3.14159265359
// #define DEG2RAD (PI/180.0)
// #define RAD2DEG (180.0/PI)

// // 機械手臂 DH 參數 (來自 tm_jacob.cpp)
// #define D1 0.1451
// #define A2 0.429
// #define A3 0.4115
// #define D4 0.1222
// #define D5 0.106
// #define D6 0.11315

// double joint[6];             // 來自 Callback 的當前角度 (deg)
// double actual_joint_vel[6];  // 來自 Callback 的當前速度 (deg/s)
// std::fstream file;           // 紀錄資料用

// // =============================================================================
// //  tm_jacobian 命名空間 (整合數學公式)
// // =============================================================================
// namespace tm_jacobian {

//     // 工具：矩陣轉 vector
//     void Matrix2DoubleVector(Eigen::MatrixXf InputMatrix, std::vector<double> &vec) {
//         Eigen::MatrixXf InputTranspose = InputMatrix.transpose();
//         int row = InputMatrix.rows();
//         int col = InputMatrix.cols();
//         for (int i = 0; i < row*col; ++i) vec[i] = InputTranspose(i);
//     }

//     // 工具：檢查速度極限
//     bool CheckVelocityLimit(std::vector<double> qd) {
//         bool valid = true;
//         double limit_1 = 180 * DEG2RAD; // 1~3 軸速限
//         double limit_2 = 225 * DEG2RAD; // 4~6 軸速限
//         if(abs(qd[0]) > limit_1 || abs(qd[1]) > limit_1 || abs(qd[2]) > limit_1) valid = false;
//         else if(abs(qd[3]) > limit_2 || abs(qd[4]) > limit_2 || abs(qd[5]) > limit_2) valid = false;
//         return valid;
//     }

//     // ------------------------------------------------------------------------
//     // [重要] 逆 Jacobian 矩陣
//     // ------------------------------------------------------------------------
//     Eigen::Matrix<float, 6, 6> Inverse_Jacobian(Eigen::Matrix<float, 6,1> q)
//     {
// 		Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero();
// 		jacobian << D6*(cos(q(0))*cos(q(4)) + sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - D4*cos(q(0)) - D5*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*cos(q(1))*sin(q(0)) - A3*cos(q(1))*cos(q(2))*sin(q(0)) + A3*sin(q(0))*sin(q(1))*sin(q(2)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - A2*cos(q(0))*sin(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))), -D6*(sin(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))),                                                                                                                                       				        						   0.,
// 	 				D5*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - D4*sin(q(0)) + D6*(cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + A2*cos(q(0))*cos(q(1)) + A3*cos(q(0))*cos(q(1))*cos(q(2)) - A3*cos(q(0))*sin(q(1))*sin(q(2)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*sin(q(0))*sin(q(1)) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))),  D6*(cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))),                                                                                                                                                      								   0.,
// 																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                             					 A3*sin(q(1))*sin(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - A3*cos(q(1))*cos(q(2)) - A2*cos(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                                   					   A3*sin(q(1))*sin(q(2)) - A3*cos(q(1))*cos(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                 				- D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                     			 -D6*cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))),                                                                                                                                                     								   0.,
// 																					                     	                                                                                                                                                                                                                                                                                                                                                                  			 0.,                                                                                                                                                                                                                                                                                                                                                  																		 -sin(q(0)),                                                                                                                                                                                                                                                                                                                            																	-sin(q(0)),                                                                                                                                                                                                                                                                  														 -sin(q(0)),                                   		 cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))),   cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))),
// 																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                                                                                                                                                                                                                                                                                   																		  cos(q(0)),                                                                                                                                                                                                                                                                                                                             																	 cos(q(0)),                                                                                                                                                                                                                                                                   														  cos(q(0)),                                   		 cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))), - cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))),
// 																					                                                                                                                                                                                                                                                                                                                                                                                       				 1.,                                                                                                                                                                                                                                                                                                                                                        																		 0.,                                                                                                                                                                                                                                                                                                                                  																		0.,                                                                                                                                                                                                                                                                        															 0.,                                                                   				 cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))),                                                    			   -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
// 		return jacobian.inverse();                  
// 	}

//     // ------------------------------------------------------------------------
//     // [重要] 正運動學 (帶夾爪長度)
//     // ------------------------------------------------------------------------
//    void Forward_Kinematics_gripper(const double* q, double* T, double Length) 
//     {
//         const double _PI_2 = 1.57079632679;
//         double c1, c2, c3, c4, c5, s1, s2, s3, s4, s5;
//         double cp, sp; // 雖然這裡可能沒用到 c6/s6，但保留變數宣告無妨
        
//         // 角度三角函數計算
//         c1 = cos(q[0]); s1 = sin(q[0]);
//         c2 = cos(q[1] - _PI_2); s2 = sin(q[1] - _PI_2);
//         c3 = cos(q[2]); s3 = sin(q[2]);
//         c4 = cos(q[3] + _PI_2); s4 = sin(q[3] + _PI_2);
//         c5 = cos(q[4]); s5 = sin(q[4]);
        
//         // 為了計算末端姿態 (RX, RY, RZ)，我們需要計算投影角度
//         // 這裡簡化處理，直接帶入 TM 原始的正運動學矩陣公式 (4x4 Matrix)
//         // T = [ R  P ]
//         //     [ 0  1 ]
        
//         // 為了節省篇幅並確保正確，這裡使用標準 TM5 的 DH 矩陣展開公式
//         // 若您之前的 tm_jacob.cpp 有更完整的 T[0]~T[15] 請以那個為主，以下是標準公式推導：

//         // 簡化變數
//         double s23 = sin(q[1] - _PI_2 + q[2]);
//         double c23 = cos(q[1] - _PI_2 + q[2]);
//         double s234 = sin(q[1] - _PI_2 + q[2] + q[3] + _PI_2);
//         double c234 = cos(q[1] - _PI_2 + q[2] + q[3] + _PI_2);

//         // 以下公式對應 TM Robot 的座標變換
//         // [注意] 由於手動展開很長，我這裡還原您原檔 tm_jacob.cpp 裡真正的 Forward_Kinematics_gripper 邏輯
        
//         double c6 = cos(q[5]); 
//         double s6 = sin(q[5]);
//         cp = cos(q[1] + q[2] + q[3]); // 用來近似
//         sp = sin(q[1] + q[2] + q[3]);

//         // --- 矩陣第一列 (R11, R12, R13, Px) ---
//         T[0]  = c1*sp*s6 - s1*s5*c6 + c1*cp*c5*c6; 
//         T[1]  = c1*sp*c6 + s1*s5*s6 - c1*cp*c5*s6; 
//         T[2]  = c1*cp*s5 + s1*c5;
//         // Px
//         T[3]  = D5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - D4*s1 + D6*(c5*s1 + s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + Length*(c5*s1 + s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + A2*c1*c2 + A3*c1*c2*c3 - A3*c1*s2*s3;

//         // --- 矩陣第二列 (R21, R22, R23, Py) ---
//         T[4]  = s1*sp*s6 + c1*s5*c6 + s1*cp*c5*c6; 
//         T[5]  = s1*sp*c6 - c1*s5*s6 - s1*cp*c5*s6; 
//         T[6]  = s1*cp*s5 - c1*c5;
//         // Py
//         T[7]  = D5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) + D4*c1 - D6*(c1*c5 + s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - Length*(c1*c5 + s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) + A2*c2*s1 + A3*c2*c3*s1 - A3*s1*s2*s3;

//         // --- 矩陣第三列 (R31, R32, R33, Pz) ---
//         T[8]  = cp*s6 - sp*c5*c6; 
//         T[9]  = cp*c6 + sp*c5*s6; 
//         T[10] = -sp*s5;
//         // Pz
//         T[11] = D1 - A2*s2 + D5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) - A3*c2*s3 - A3*c3*s2 - D6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - Length*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));

//         // --- 矩陣第四列 ---
//         T[12] = 0; 
//         T[13] = 0; 
//         T[14] = 0;
//         T[15] = 1;
//     }

//     // ------------------------------------------------------------------------
//     // [核心] 計算關節速度指令
//     // ------------------------------------------------------------------------
//     bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition, std::vector<double> EFF_Velocity, std::vector<double>& qd)
//     {
//         Eigen::Matrix<float, 6, 1> home, q;
//         Eigen::Matrix<float, 6, 1> effspd, jointspd;

//         // TM 機械手臂的 Home Offset (定義於 tm_jacob.cpp)
//         home << 0, -PI*0.5, 0, PI*0.5, 0, 0;
        
//         effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
//         q << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
        
//         // 補償 Offset
//         q += home;

//         // 計算 V_joint = J_inverse * V_cartesian
//         Eigen::Matrix<float, 6, 6> Inv_J = Inverse_Jacobian(q);
//         jointspd = Inv_J * effspd;

//         Matrix2DoubleVector(jointspd, qd);

//         return CheckVelocityLimit(qd);
//     }
// }

// // =============================================================================
// //  輔助函式：工具類
// // =============================================================================

// // 讀取 CSV
// std::vector<std::vector<float>> read_csv(std::string filename){
//     std::vector<std::vector<float>> content;
//     std::vector<float> row;
//     std::string line, word;
//     std::fstream file (filename, std::ios::in);
//     if(file.is_open()) {
//         while(getline(file, line)) {
//             row.clear();
//             std::stringstream str(line);
//             while(getline(str, word, ','))
//                 row.push_back(stof(word));
//             content.push_back(row);
//         }
//     } else {
//         std::cout<<"Could not open the file: " << filename << "\n";
//     }
//     return content;
// }

// // 矩陣轉 RPY (XYZ Euler Angle)
// void Matrix2RPY(double* T, std::vector<double> &pose) {
//     // 位置 X, Y, Z
//     pose[0] = T[3]; 
//     pose[1] = T[7]; 
//     pose[2] = T[11]; 
    
//     // 旋轉矩陣轉 Euler (ZYX順序，對應 Yaw-Pitch-Roll)
//     // 這裡使用 Eigen 簡化計算
//     Eigen::Matrix3f rot;
//     rot << T[0], T[1], T[2],
//            T[4], T[5], T[6],
//            T[8], T[9], T[10];
           
//     // 使用 Euler Angles (0,1,2 代表 X, Y, Z 軸)
//     // 注意：需確認 TM 的 RxRyRz 定義，通常接近 Euler ZYX
//     Eigen::Vector3f euler = rot.eulerAngles(0, 1, 2); 
//     pose[3] = euler[0]; // Rx
//     pose[4] = euler[1]; // Ry
//     pose[5] = euler[2]; // Rz
// }

// // 角度正規化 (-PI ~ PI)
// double normalize_angle(double angle) {
//     while (angle > PI) angle -= 2.0 * PI;
//     while (angle < -PI) angle += 2.0 * PI;
//     return angle;
// }

// // ROS Callback
// void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
// {
//     if(msg->joint_pos.size() == 6) {
//         for (int i = 0; i < 6; i++)
//             joint[i] = std::atof(std::to_string(msg->joint_pos[i]).c_str()) / PI * 180.0; // 存成 deg
//     }
//     if(msg->joint_vel.size() == 6) {
//         for (int i = 0; i < 6; i++)
//             actual_joint_vel[i] = msg->joint_vel[i] * 180.0 / PI; // 存成 deg/s
//     }
// }

// // =============================================================================
// //  Main Function
// // =============================================================================
// int main(int argc, char **argv)
// {   
//     // 1. 初始化
//     ros::init(argc, argv, "demo_cartesian_control");      
//     ros::NodeHandle nh_demo; 
//     ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
//     ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
//     tm_msgs::SendScript srv;

//     // 2. 讀取路徑資料
//     std::vector<std::vector<float>> data = read_csv("/home/lab816/agv_ws/src/tmr_ros1/demo/data_setting/x3.5cm.csv");
//     if (data.empty()) { ROS_ERROR("CSV data is empty!"); return -1; }

//     // 3. 設定參數
//     int count = 0;
//     float speed_saturation = 40.0; // 稍微放寬一點，因為 Inverse Jacobian 計算出的值變化較大
//     float sampling_rate = 100.0; 
//     ros::Rate rate(sampling_rate);

//     // 4. 回到初始點
//     std::string cmd_init = "PTP(\"JPP\",";
//     for (int i = 0; i < 6; i++) cmd_init += std::to_string(data[0][i]) + ",";
//     cmd_init += "80,100,0,false)";

//     srv.request.id = "demo"; srv.request.script = cmd_init;
//     if (client.call(srv) && srv.response.ok) ROS_INFO("Moving to Init Position...");
//     else { ROS_ERROR("Failed to send init script"); return 1; }
    
//     ros::Duration(3.0).sleep(); // 等待到達

//     // 5. 啟動速度模式
//     srv.request.script = "ContinueVJog()";
//     client.call(srv);
//     ROS_INFO("Velocity Mode Started.");

//     // 6. 開檔紀錄
//     file.open("/home/lab816/PID_Cartesian_Log.csv", std::ios::out | std::ios::trunc);
//     file << "count,err_x,err_y,err_z,err_rx,err_ry,err_rz,v_cmd_0,v_cmd_1,v_cmd_2,v_cmd_3,v_cmd_4,v_cmd_5\n";

//     // 7. 控制變數宣告
//     std::vector<double> current_q_rad(6), desired_q_rad(6);
//     std::vector<double> current_pose(6), desired_pose(6); // [X,Y,Z,Rx,Ry,Rz]
//     double T_curr[16], T_des[16];
//     std::vector<double> v_cartesian(6);
//     std::vector<double> qd_cmd(6);
//     float command[6] = {0};

//     // PID 增益 (卡式空間)
//     float kp_pos = 2.0; // 對應 XYZ 誤差 (單位 1/s)
//     float kp_rot = 1.0; // 對應 RPY 誤差

//     // ========================== Control Loop ==========================
//     while (ros::ok()){
//         ros::spinOnce(); 

//         if (count >= data.size()) {
//             ROS_INFO("Path Finished.");
//             srv.request.script = "StopContinueVmode()";
//             client.call(srv);
//             break;
//         }

//         // A. 準備資料 (Deg -> Rad)
//         for(int i=0; i<6; i++) {
//             current_q_rad[i] = joint[i] * DEG2RAD;       // 當前 Robot 角度
//             desired_q_rad[i] = data[count][i] * DEG2RAD; // CSV 內的目標角度
//         }

//         // B. 正運動學 (Joint -> Cartesian)
//         // 計算「目標在哪」以及「我現在在哪」
//         tm_jacobian::Forward_Kinematics_gripper(current_q_rad.data(), T_curr, 0.0);
//         tm_jacobian::Forward_Kinematics_gripper(desired_q_rad.data(), T_des, 0.0);

//         Matrix2RPY(T_curr, current_pose);
//         Matrix2RPY(T_des, desired_pose);

//         // C. 計算卡式誤差與速度指令 (P Control)
//         // XYZ
//         for(int i=0; i<3; i++) {
//             v_cartesian[i] = kp_pos * (desired_pose[i] - current_pose[i]);
//         }
//         // RPY (需正規化角度誤差)
//         for(int i=3; i<6; i++) {
//             double err = desired_pose[i] - current_pose[i];
//             v_cartesian[i] = kp_rot * normalize_angle(err);
//         }

//         // D. 逆 Jacobian (Cartesian Velocity -> Joint Velocity)
//         bool result = tm_jacobian::GetQdfromInverseJacobian(current_q_rad, v_cartesian, qd_cmd);

//         // E. 發送指令 (Rad/s -> Deg/s + 限速)
//         std::string script = "SetContinueVJog(";
//         for(int i=0; i<6; i++) {
//             double vel_deg = qd_cmd[i] * RAD2DEG;

//             if(vel_deg > speed_saturation) vel_deg = speed_saturation;
//             if(vel_deg < -speed_saturation) vel_deg = -speed_saturation;
            
//             command[i] = vel_deg;
//             script += std::to_string(command[i]);
//             if(i < 5) script += ",";
//         }
//         script += ")";

//         srv.request.script = script;
//         client.call(srv);

//         // F. 紀錄與更新
//         file << count << "," 
//              << (desired_pose[0]-current_pose[0]) << ","
//              << (desired_pose[1]-current_pose[1]) << ","
//              << (desired_pose[2]-current_pose[2]) << ","
//              << normalize_angle(desired_pose[3]-current_pose[3]) << ","
//              << normalize_angle(desired_pose[4]-current_pose[4]) << ","
//              << normalize_angle(desired_pose[5]-current_pose[5]);
//         for(int i=0; i<6; i++) file << "," << command[i];
//         file << "\n";

//         count++;
//         rate.sleep();
//     }
    
//     file.close();
    
//     // 結束確認
//     std::cout << "Press [Enter] to return to Start Position..." << std::endl;
//     std::cin.get();
//     srv.request.script = cmd_init;
//     client.call(srv);

//     return 0;
// }