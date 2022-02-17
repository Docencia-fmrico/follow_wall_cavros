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


#include "follow_wall_cavros/LifeCycle.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "follow_wall_cavros/LaserNode.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto laser_node = std::make_shared<follow_wall_cavros::LaserNode>("Laser_Node", 1ms);
  auto life_node = std::make_shared<LifeCycle>("LifeCycle", 1ms);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_node);
  executor.add_node(life_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
