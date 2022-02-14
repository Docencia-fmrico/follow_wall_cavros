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

//#include "rclcpp/rclcpp.hpp"

#include "follow_wall_cavros/MoveNode.hpp"
#include "follow_wall_cavros/LaserNode.hpp"
#include "follow_wall_cavros/LifeCycle.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // trigger_transition para transicionar en el lifecycle.
  // executors + lifecycles = node->get_node_base_interface()

  //auto --- legibilidad del codigo aumenta si es evidente lo que hay a la derecha
  //auto node = rclcpp::Node::make_shared("simple_node"); //constructor del nodo y le pasas su nombre
  auto pub_node = std::make_shared<follow_wall_cavros::MoveNode>("Move_node");
  auto sub_node = std::make_shared<follow_wall_cavros::LaserNode>("Laser_Node");
  auto life_node = std::make_shared<LifeCycle>();

  rclcpp::Rate loop_rate(300ms);

  while (rclcpp::ok()) {
    life_node->do_work();
    // pub_node->pub_vel();
    // rclcpp::spin_some(pub_node);
    // rclcpp::spin_some(sub_node);
    rclcpp::spin_some(life_node->get_node_base_interface());
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
