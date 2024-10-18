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

#ifndef OMNI_DRIVE_GAZEBO_HPP_
#define OMNI_DRIVE_GAZEBO_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
// #include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class OmniDrive : public rclcpp::Node
{
public:
  OmniDrive();
  ~OmniDrive();

private:
  // ROS topic publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ROS topic subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_vel_pub_;

  // Variables
  geometry_msgs::msg::Twist cmd_vel_;
  double robot_r; // Robot radius
  double wheel_r; // Wheel radius

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_joint_vels(double front, double left, double right);
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
#endif  // OMNI_DRIVE_GAZEBO_HPP_
