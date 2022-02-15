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

#define ANGULAR_VEL 0.2
#define LINEAR_VEL 0.2

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
    
    //linear
    if (fabs(angle) < 10) {
      x_ = LINEAR_VEL * 1.5;
    } else if (fabs(angle) < 20) {
      x_ = LINEAR_VEL;
    } else {
      x_ = LINEAR_VEL / 2;
    }
    
    //angular
    if (angle > 10) {
      z_ = ANGULAR_VEL; // rotate left to face wall
    } else if (angle < -10) {
      z_ = -ANGULAR_VEL; // rotate right to face wall
    } else {
      z_ = 0.0; // aproach wall
    }
  /*
    
    if (angle > 10) {
      z_ = ANGULAR_VEL;  // rotate left to face wall
      if (angle < 20) {
        x_ = LINEAR_VEL;
      } else {
        x_ = LINEAR_VEL/2;
      }
    } else if (angle < -10) {
      z_ = -ANGULAR_VEL;  // rotate right to face wall
      if (angle > -20) {
        x_ = LINEAR_VEL;
      } else {
        x_ = LINEAR_VEL/2;
      }
    } else {  // going wall
      z_ = 0.0;
      x_ = LINEAR_VEL * 1.5;
    }

    */
  } else {  // already close to wall
    
    // lineal
    if (angle > -105 && angle < -75) {  // parallel to wall
      x_ = LINEAR_VEL + 0.1;
    } else if (min_distance < 0.3) {  // too close to wall go back
      x_ = -LINEAR_VEL / 6;
    } else {
      x_ = LINEAR_VEL/6;
    }

    // angular
    if (angle < -95) {  // turn right
      z_ = -ANGULAR_VEL;
    } else if(angle > -85) {  // turn left
      z_ = ANGULAR_VEL;
    } else {
      z_ = 0.0;
    }

    if (angle > -110 && angle < -70 && right_distance > 2.5) {  // detect open door
      x_ = 0.4;//LINEAR_VEL-0.05;
      z_ = -0.4;//- ( (LINEAR_VEL/1.8) / min_distance); //-(ANGULAR_VEL*1.7);
    }
  }
}
}  // namespace follow_wall_cavros
