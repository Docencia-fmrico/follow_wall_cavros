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

#ifndef FOLLOW_WALL_CAVROS__LIFECYCLE_HPP_
#define FOLLOW_WALL_CAVROS__LIFECYCLE_HPP_

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "follow_wall_cavros/MoveNode.hpp"
#include "follow_wall_cavros/LaserNode.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;  // 500ms...

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class LifeCycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifeCycle();
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  void do_work();
private:
  std::shared_ptr<follow_wall_cavros::MoveNode> pub_node_;
  std::shared_ptr<follow_wall_cavros::LaserNode> sub_node_;
};

#endif  // FOLLOW_WALL_CAVROS__LIFECYCLE_HPP_
