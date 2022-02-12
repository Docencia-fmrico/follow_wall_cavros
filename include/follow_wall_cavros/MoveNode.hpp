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

//#ifndef FOLLOW_WALL_CAVROS__MOVENODE_HPP_
//#define FOLLOW_WALL_CAVROS__MOVENODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;//500ms...

namespace follow_wall_cavros{
  class MoveNode : public rclcpp::Node
  {
  public:
    MoveNode(const std::string & name);
    void pub_vel(void);

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_; // publicador de velocidades  
  };
}

