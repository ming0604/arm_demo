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
#include <signal.h> // For ctrl+c handling

#include <sys/sem.h>
#include <sys/ipc.h>
#include <unistd.h>

std::fstream file;
std::fstream file_start;
std::fstream PID_file;
// TM Driver header
#include "tm_msgs/SendScript.h"

#define pi 3.1415926

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Global variable declaration
double joint[6];
double actual_joint_vel[6]; // Store actual joint velocities (deg/s)

bool CtrlCStop = false;

key_t semKey_rend_sync = (key_t)555; //semaphore key
struct sembuf sop; // for P() V() operation
int semid_rend_sync;

// =============== functions ===================
int P(int s); // aquire semaphore (try to decrease) 
int V(int s); // release semaphore (increase)
int P_idx(int s, int index); // aquire semaphore at index in semaphore set
int V_idx(int s, int index); // release semaphore at index in semaphore set

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

void my_sigint_handler(int signum)
{
    CtrlCStop = true;
}

// Callback function
void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  // Read position
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
  ros::init(argc, argv, "demo_send_script_sync", ros::init_options::NoSigintHandler); 
    // handle Ctrl+C
  signal(SIGINT, my_sigint_handler);

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
  // float speed_saturation = 20.0;
  float speed_saturation = 45.0;
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
  
  // CSV header
  file << "count,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],desired[0],desired[1],desired[2],desired[3],desired[4],desired[5],v[0],v[1],v[2],v[3],v[4],v[5],act_v[0],act_v[1],act_v[2],act_v[3],act_v[4],act_v[5]\n";

  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();

  // access existing semaphore created by platform controller
  if((semid_rend_sync = semget(semKey_rend_sync, 0, 0)) == -1) 
  {
      printf("semget(rendezvous sem set) error\n");
      return -1;
  }

  // need to wait for the platform controller to be ready
  printf(ANSI_COLOR_GREEN "Waiting for platform controller to be ready...\n" ANSI_COLOR_RESET);
  //============= use rendezvous synchronization =======================
  V_idx(semid_rend_sync, 1); // release semaphore 2 (index 1) (I am ready)
  P_idx(semid_rend_sync, 0); // aquire semaphore 1 (index 0) (wait for platform controller to be ready)
  printf(ANSI_COLOR_GREEN "Get rendezvous semaphore 1. Platform controller is ready.\n" ANSI_COLOR_RESET);

  ROS_INFO_STREAM("Starting Control Loop...");
  usleep(10000); // sleep for 10ms to wait for platform controller's first timer routine
  // ========================== PID Control Loop ==========================
  while (ros::ok() && !CtrlCStop){
    ros::spinOnce(); 
    
    //bool path_finished = false; 

    // Display desired joint (can be commented out for cleaner output)
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
        // Read CSV target point
        if (count >= data.size()){
            for(int i=0; i<6; i++) desired[i] = goal_joint[i];
            //path_finished = true; 
            remove("/home/lab816/Desktop/code_cheng/code/main/mainKinematic/check_file/start.txt");
        }
        else{
            for(int i=0; i<6; i++) desired[i] = data[count][i];
        }

        // Calculate Error and Command
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

        // Check if reached
        /*
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
                
                // 1. Stop velocity mode
                srv.request.id = "demo";
                srv.request.script = cmd_stop;
                client.call(srv);
                ROS_INFO_STREAM("Velocity Mode Stopped.");
                
                // 2. Close file
                file.close();
                if (file_start.is_open()) file_start.close();

                // 3. Break Control Loop (Important!)
                break; 
            }
        }
        */

        // Send velocity command
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

  // Cleanup when loop exits (e.g. Ctrl+C triggers !ros::ok())
  // Stop velocity mode
  srv.request.id = "demo";
  srv.request.script = cmd_stop;
  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Stop Velocity Mode => Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error sending stop script to robot (Connection might be closed due to Ctrl+C)");
    //return 1;
  }

  if (file.is_open()) {
      file.close(); 
      ROS_INFO_STREAM("Output file closed.");  
  }
  
  if (file_start.is_open()) {
      file_start.close();
  }

  ROS_INFO_STREAM("Program Finished.");
  return 0;
}




//========================================================================
//                          semaphore operations
//========================================================================
int P(int s) // aquire (try to decrease)
{
	sop.sem_num = 0; 
	sop.sem_op = -1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (P(sem))\n");
		return -1;
	}
	return 0;
}

int V(int s) // release (increase)
{
	sop.sem_num = 0;
	sop.sem_op = 1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (V(sem))\n");
		return -1;
	}
	return 0;
}

int P_idx(int s, int index) // aquire semaphore at index in semaphore set
{
    sop.sem_num = index;     // select which semaphore in the semaphore set
    sop.sem_op = -1;   
    sop.sem_flg = 0;

    if(semop(s, &sop, 1) < 0)
    {
        printf("semop error: (P_idx(sem, idx))\n");
        return -1;
    }
    else
    {
        return 0;
    }   
}

int V_idx(int s, int index) // release semaphore at index in semaphore set
{
    sop.sem_num = index;    // select which semaphore in the semaphore set
    sop.sem_op = 1;
    sop.sem_flg = 0;

    if(semop(s, &sop, 1) < 0)
    {
        printf("semop error: (V_idx(sem, idx))\n");
        return -1;
    }
    else
    {
        return 0;
    }
}