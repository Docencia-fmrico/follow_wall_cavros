// Copyright 2020 Intelligent Robotics Lab
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

#include <string>

#include "follow_wall_cavros/LaserNode.hpp"

#define DOOR_ANGLE -(M_PI/3)*2

#define UP -M_PI
#define DOWN 0
#define LEFT M_PI/2
#define RIGHT -M_PI/2

using namespace std::chrono_literals;  // 500ms...

namespace follow_wall_cavros
{
LaserNode::LaserNode(const std::string & name, const std::chrono::nanoseconds & rate)
: Node(name)
{
  sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_filtered", 10, std::bind(&LaserNode::Laser_callback, this, _1));
  laser_info_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("laser_info", 10);

  timer_ = create_wall_timer(
    rate, std::bind(&LaserNode::publish_laser_info, this));
}

void LaserNode::Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Laser Range of distances : min = 0.05m ; max = 25m
  // data in msg->ranges goes from right to left
  // there are 665 values

  int pos, j = 0, n_positions, door_position;

  // angle range follows:
  //            ^ 0º
  //            |
  // 90º <--  robot  --> -90º ( ranges[0] = -108º ; and last ranges[665] = 108º)
  // each angle has 3.055 values

  // -- TRAZAS -- 
  // int up_idx = (UP - msg->angle_min) / msg->angle_increment;
  // int down_idx = (DOWN - msg->angle_min) / msg->angle_increment;
  // int left_idx = (LEFT - msg->angle_min) / msg->angle_increment;
  // int right_idx = (RIGHT - msg->angle_min) / msg->angle_increment;

  // float up_angle = ((up_idx * msg->angle_increment) + msg->angle_min) * 180/M_PI;
  // float down_angle = ((down_idx * msg->angle_increment) + msg->angle_min) * 180/M_PI;
  // float left_angle = ((left_idx * msg->angle_increment) + msg->angle_min) * 180/M_PI;
  // float right_angle = ((right_idx * msg->angle_increment) + msg->angle_min) * 180/M_PI;

  // RCLCPP_INFO(this->get_logger(), " ---- ");
  // RCLCPP_INFO(this->get_logger(), "[UP] Angle [%f] Index [%d] Distance [%f]", up_angle, 0, msg->ranges[0]);
  // RCLCPP_INFO(this->get_logger(), "[DOWN] Angle [%f] Index [%d] Distance [%f]", down_angle, down_idx, msg->ranges[down_idx]);
  // RCLCPP_INFO(this->get_logger(), "[LEFT] Angle [%f] Index [%d] Distance [%f]", left_angle, left_idx, msg->ranges[left_idx]);
  // RCLCPP_INFO(this->get_logger(), "[RIGHT] Angle [%f] Index [%d] Distance [%f]", right_angle, right_idx, msg->ranges[right_idx]);
  // RCLCPP_INFO(this->get_logger(), " ---- ");

  door_position = (DOOR_ANGLE - msg->angle_min) / msg->angle_increment;
  door_distance_ = msg->ranges[door_position];

  n_positions = (msg->angle_max - msg->angle_min) / msg->angle_increment;
  min_ = 25;
  for (int i = 0; i < n_positions; i++) {
    if (!(std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i]))) {
      // min distance
      if (msg->ranges[i] < min_) {
        min_ = msg->ranges[i];
        pos = i;
      }
    }
  }
  angle_ = ((pos * msg->angle_increment) + msg->angle_min) * 180 / M_PI;
}

void LaserNode::publish_laser_info(void)
{
  std_msgs::msg::Float32MultiArray info;

  info.data.push_back(angle_);  // angle respect to min distance
  info.data.push_back(min_);
  info.data.push_back(door_distance_);  // laser value to detect open doors

  laser_info_pub_->publish(info);
}

float LaserNode::get_angle(void)
{
  return angle_;
}

float LaserNode::get_min_distance(void)
{
  return min_;
}

bool LaserNode::door_open(void)
{
  return door_distance_ > 2.5;
}

}  // namespace follow_wall_cavros
