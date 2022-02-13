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

using namespace std::chrono_literals;//500ms...
using std::placeholders::_1;

namespace follow_wall_cavros{
  MoveNode::MoveNode(const std::string & name)
    : Node(name)
    {
      vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
      dist_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("laser_info", 10, std::bind(&MoveNode::distance_callback, this, _1));
    }

  void MoveNode::pub_vel(void)
  {
    geometry_msgs::msg::Vector3 vel;
    geometry_msgs::msg::Twist msg_vel;

    vel.x = x_;
    vel.y = y_;
    vel.z = z_;

    msg_vel.linear = vel;

    vel_pub_->publish(msg_vel);
  }

  //void MoveNode::follow_wall(){}

  void MoveNode::distance_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    
    float min_distance = msg->data[0], angle = msg->data[1];
    bool parallel = false;

    if( (angle <= 100 && angle >= 80) || ( angle <= -80 && angle >= -100 )){
      parallel = true;
    }
    RCLCPP_INFO(this->get_logger(),"dist: %f ; angle : %f ; parallel?= %d\n",min_distance,angle,parallel);

    if(min_distance < 1 && parallel){
      //MoveNode::follow_wall();
    }else{
      if(angle > -10 && angle < 10){
        x_ = 0.5;
      }else{
        x_ = 0.0;
        if(angle < 0){
          z_ = 1.0;
        }else{
          z_ = -1.0;
        }
      }
    } 
  }

}// namespace 

