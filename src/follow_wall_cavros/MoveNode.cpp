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

namespace follow_wall_cavros{
  MoveNode::MoveNode(const std::string & name)
    : Node(name)
    {
      vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
    }

  void MoveNode::pub_vel(void)
  {

    geometry_msgs::msg::Vector3 vel;
    geometry_msgs::msg::Twist msg_vel;

    vel.x = 0.0;
    vel.y = 0.0;
    vel.z = 0.0;

    msg_vel.linear = vel;

    vel_pub_->publish(msg_vel);
  }
}

