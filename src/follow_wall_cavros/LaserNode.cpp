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

#include "follow_wall_cavros/LaserNode.hpp"

#define DOOR_ANGLE -M_PI/6

using namespace std::chrono_literals;  // 500ms...

namespace follow_wall_cavros
{
LaserNode::LaserNode(const std::string & name)
: Node(name)
{
  sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10, std::bind(&LaserNode::Laser_callback, this, _1));
  laser_info_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("laser_info", 10);
}

void LaserNode::Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Laser Range of distances : min = 0.05m ; max = 25m
  // data in msg->ranges goes from right to left
  // there are 665 values

  std_msgs::msg::Float32MultiArray info;
  int pos, j = 0, n_positions, door_position;

  // angle range follows:
  //            ^ 0º
  //            |
  // 90º <--  robot  --> -90º ( ranges[0] = -108º ; and last ranges[665] = 108º)
  // each angle has 3.055 values

  door_position = (DOOR_ANGLE - msg->angle_min) / msg->angle_increment;
  door_distance_ = msg->ranges[door_position];

  n_positions = (msg->angle_max - msg->angle_min) / msg->angle_increment;
  min_ = 25;
  for (int i = 0; i < n_positions; i++) {
    // min distance
    if (msg->ranges[i] < min_) {
      min_ = msg->ranges[i];
      pos = i;
    }
  }

  angle_ = ( pos * msg->angle_increment) + msg->angle_min;
  angle_ = angle_ * 180/M_PI;

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
