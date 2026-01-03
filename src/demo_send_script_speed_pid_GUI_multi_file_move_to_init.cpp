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
#include <iomanip> // For std::setprecision
#include <iostream> // Include iostream

std::fstream file;
std::fstream file_start;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"

#define pi 3.1415926

// global variables
double joint[6];            // store actual joint positions (deg)
double actual_joint_vel[6]; // store actual joint velocities (deg/s)

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

// Callback function to read joint positions and velocities
void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  // Read joint positions
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

  // Read velocity
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
  ros::init(argc, argv, "demo_move_to_init");      
  ros::NodeHandle nh_demo; 
  ros::NodeHandle nh_private("~"); // for getting private parameters
  ros::Subscriber sub = nh_demo.subscribe("feedback_states", 1000, TMmsgCallback);
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  std::string csv_path;
  std::string PTP_speed;
  // get parameters from launch file or command line
  nh_private.param<std::string>("traj_csv_path", csv_path, "/Default/path");
  nh_private.param<std::string>("speed_percent", PTP_speed, "80");
  
  // Limit speed to 1~100
  int speed_val = std::stoi(PTP_speed);
  if(speed_val > 100) 
  {
    speed_val = 100;
    ROS_WARN("Speed parameter > 100, clamped to 100.");
  } 
  else if(speed_val < 1) 
  {
    speed_val = 1;
    ROS_WARN("Speed parameter < 1, clamped to 1.");
  }
  PTP_speed = std::to_string(speed_val);

  std::cout << "CSV Path: " << csv_path << std::endl;
  std::cout << "PTP speed(%): " << PTP_speed << std::endl;

  std::vector<std::vector<float>> data;

  int count = 0;
  float speed_saturation = 20.0;
  // Tolerance (deg)
  float tolerance = 0.5; 
  float init_joint[6];
 
  
  // read the desired joint positions from CSV file
  data = read_csv(csv_path);
  
  if (data.empty()) {
      ROS_ERROR("CSV data is empty or file not found!");
      return -1;
  }

  // Create PTP command string to move to the initial point
  std::string cmd_init = "PTP(\"JPP\",";
  std::string PTP_para = PTP_speed + ",100,0,false)";

  for (int i = 0; i < 6; i++){
    init_joint[i] = data[0][i];
    cmd_init += std::to_string(data[0][i]) + ",";
  }
  
  cmd_init += PTP_para;
  //cmd_init += "80,100,0,false)";
  std::cout << "data size: " << data.size() << ", " << data[0].size() << std::endl;
  std::cout << "cmd_init: "<< cmd_init << std::endl;



  float sampling_rate = 100; // Hz

  ros::Rate rate(sampling_rate);
  
  // 1. Move to initial point first (when program starts)
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

  auto start_time = std::chrono::steady_clock::now();
  ROS_INFO_STREAM("Move to initial position command sent. Waiting for robot to reach the position...");
  
  // === Loop to check if position reached ===
  bool reached = false;

  while(ros::ok()) 
  {
      ros::spinOnce();
      
      double max_error = 0.0;
      for(int i=0; i<6; i++)
      {
        double err = std::abs(init_joint[i] - joint[i]);
        if(err > max_error) 
        {
          max_error = err;
        }
      }

      // Display current max error
      ROS_INFO_THROTTLE(2.0, "Current max position error: %.4f deg", max_error);

      if(max_error < tolerance) 
      {
        reached = true;
        break; // Break loop
      }

      rate.sleep();
  }

  if(reached) 
  {
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;  // Defalt unit is seconds
    ROS_INFO("Robot has reached the initial position within the angle tolerance %f.", tolerance);
    ROS_INFO("Time taken: %f seconds", elapsed_seconds.count());
  }

  return 0;
}
  