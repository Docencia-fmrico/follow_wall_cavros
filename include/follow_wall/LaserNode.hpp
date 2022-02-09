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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LaserNode : public rclcpp::Node
{
public:
  LaserNode(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name)
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("velocities", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/LaserScan", 10, std::bind(&LaserNode::callback, this, _1));
  }

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; // publicador de velocidades
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_; // subscriptor del laser

  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;
  void publishVel(void);
};
