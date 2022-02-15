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

#ifndef FOLLOW_WALL_CAVROS__LASERNODE_HPP_
#define FOLLOW_WALL_CAVROS__LASERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <string>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;  // 500ms...

namespace follow_wall_cavros
{
class LaserNode : public rclcpp::Node
{
public:
  LaserNode(const std::string & name, const std::chrono::nanoseconds & rate);
  void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void laser_pub_callback();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr laser_info_pub_;
  
  //for testing:
  float min_;
  float angle_;
  float door_distance_;

  float get_angle(void);
  float get_min_distance(void);
  bool door_open(void);

  // timers
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Float32MultiArray info_;

};
}  // namespace follow_wall_cavros

#endif  // FOLLOW_WALL_CAVROS__LASERNODE_HPP_
