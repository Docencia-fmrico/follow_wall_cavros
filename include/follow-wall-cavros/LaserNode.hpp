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

//#ifndef FOLLOW_WALL_CAVROS__LASERNODE_HPP_
//#define FOLLOW_WALL_CAVROS__LASERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;//500ms...

namespace LaserNode{


  class LaserNode : public rclcpp::Node
  {
  public:
    LaserNode(const std::string & name)
    : Node(name)
    {
      vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
      laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_raw", 10, std::bind(&LaserNode::Laser_callback, this, _1));
    }

    void pub_vel(void);

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_; // publicador de velocidades
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; // subscriptor del laser

    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;
    
  };

} //namespace LaserNode
=======
} //namespace LaserNode
