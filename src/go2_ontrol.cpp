/**
 * @file move_test.cpp
 * @brief Use the api named Move to control the vel of robot
 * @date 2023-12-20
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>

// Global variables for SportClient and velocity
unitree::robot::SportClient sport_client;
double linear_x = 0.0;
double linear_y = 0.0;
double angular_z = 0.0;

// Callback function for /cmd_vel topic
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  linear_x = msg->linear.x;
  linear_y = msg->linear.y;
  angular_z = msg->angular.z;

  // Use api to move
  sport_client.Move(linear_x, linear_y, angular_z);
}

// Forced close progress
void MySigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char** argv)
{ 
  // Initialize ROS node
  ros::init(argc, argv, "vel_move_node");
  ros::NodeHandle nh;

  // Set up signal handler for forced close
  signal(SIGINT, MySigintHandler);
  
  // Init channel
  unitree::robot::ChannelFactory::Instance()->Init();

  // Set time-out period for request
  sport_client.SetTimeout(10.0f);
  
  // Init SportClient
  sport_client.Init();
  
  // Use api to balance stand  
  sport_client.BalanceStand();
  
  sleep(3);

  // Create a subscriber for /cmd_vel topic
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

  // Spin to keep the node running and process callbacks
  ros::spin();

  // Stop move when the node is shutting down
  sport_client.StopMove();
  
  return 0;
}
