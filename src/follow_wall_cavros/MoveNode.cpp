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

#include "follow_wall_cavros/MoveNode.hpp"

using namespace std::chrono_literals;  // 500ms...
using std::placeholders::_1;

namespace follow_wall_cavros
{
MoveNode::MoveNode(const std::string & name)
: Node(name)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
  dist_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "laser_info", 10, std::bind(&MoveNode::distance_callback, this, _1));
}

void MoveNode::pub_vel(void)
{
  geometry_msgs::msg::Vector3 vel_linear, vel_angular;
  geometry_msgs::msg::Twist msg_vel;

  vel_linear.x = x_;
  vel_angular.z = z_;

  msg_vel.linear = vel_linear;
  msg_vel.angular = vel_angular;

  vel_pub_->publish(msg_vel);
}

float MoveNode::return_linear()
{
  return x_;
}

float MoveNode::return_angular()
{
  return z_;
}

void MoveNode::distance_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  float angle = msg->data[0], min_distance = msg->data[1], right_distance = msg->data[2];
  bool parallel = false;

  // aproach to wall
  if (min_distance > 0.7) {
    if (angle > 10) {
      z_ = 0.2;  // rotate left to face wall
      if (angle < 20) {
        x_ = 0.2;
      } else {
        x_ = 0.0;
      }
    } else if (angle < -10) {
      z_ = -0.2;  // rotate right to face wall
      if (angle > -20) {
        x_ = 0.2;
      } else {
        x_ = 0.0;
      }
    } else {  // going wall
      z_ = 0.0;
      x_ = 0.35;
    }
  } else {  // already close to wall
    // lineal
    if (angle > -105 && angle < -75) {  // parallel to wall
      x_ = 0.3;
    } else if (min_distance < 0.3) {  // too close to wall go back
      x_ = -0.05;
    } else {
      x_ = 0;
    }

    // angular
    if (angle < -95) {  // turn right
      z_ = -0.2;
    } else if(angle > -85) {  // turn left
      z_ = 0.2;
    } else {
      z_ = 0.0;
    }

    if (right_distance > 2.5) {  // detect open door
      z_ = -0.3;
      x_ = 0.3;
    }
  }
}
}  // namespace follow_wall_cavros
