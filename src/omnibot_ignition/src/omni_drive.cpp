// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "omnibot_ignition/omni_drive.hpp"

#include <memory>
#include <cmath>

using namespace std::chrono_literals;

OmniDrive::OmniDrive()
: Node("Omni_drive")
{
  /************************************************************
  ** Initialise parameters
  ************************************************************/
  this->declare_parameter("robot_radius", 0.251); // Unit: m
  this->declare_parameter("wheel_radius", 0.06); // Unit: m
  robot_r = this->get_parameter("robot_radius").as_double();
  wheel_r = this->get_parameter("wheel_radius").as_double();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  front_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("front_vel", qos);
  left_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_vel", qos);
  right_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_vel", qos);

  // Initialise subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos, std::bind(&OmniDrive::twist_callback, this, std::placeholders::_1));

  // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "odom", qos, std::bind(&OmniDrive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&OmniDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Omni_drive node has been initialised");
}

OmniDrive::~OmniDrive()
{
  RCLCPP_INFO(this->get_logger(), "Omni_drive node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void OmniDrive::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // x and y axis are in robot frame
  cmd_vel_.linear.x = msg->linear.x;
  cmd_vel_.linear.y = msg->linear.y;
  cmd_vel_.angular.z = msg->angular.z;
}

/********************************************************************************
** Update joint velocity commands to ignition
********************************************************************************/
void OmniDrive::update_joint_vels(double front, double left, double right)
{
  std_msgs::msg::Float64 front_vel;
  std_msgs::msg::Float64 left_vel;
  std_msgs::msg::Float64 right_vel;

  front_vel.data = front;
  left_vel.data = left;
  right_vel.data = right;
  
  front_vel_pub_->publish(front_vel);
  left_vel_pub_->publish(left_vel);
  right_vel_pub_->publish(right_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void OmniDrive::update_callback()
{
  double vx = cmd_vel_.linear.x;
  double vy = cmd_vel_.linear.y;
  double wz = cmd_vel_.angular.z;

  double wf, wl, wr;
  double cos_pi_6 = 2.0/sqrt(3.0);
  double sin_pi_6 = 0.5;

  // Inverse kinematics of 3-wheel omni drive robots
  wf = -(-vy + robot_r*wz)/wheel_r;
  wl = (-vx*cos_pi_6 + vy*sin_pi_6 + robot_r*wz)/wheel_r;
  wr = (vx*cos_pi_6 + vy*sin_pi_6 + robot_r*wz)/wheel_r;
  
  update_joint_vels(wf, wl, wr);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniDrive>());
  rclcpp::shutdown();

  return 0;
}
