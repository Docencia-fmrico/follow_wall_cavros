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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
using namespace std::chrono_literals;//500ms...


class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher()
  : Node("componsable_node_pub")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
  }

  void pub_vel()
  {
    geometry_msgs::msg::Vector3 vel;
    geometry_msgs::msg::Twist msg_vel;

    vel.x = -3.0;
    vel.y = 0.0;
    vel.z = 0.0;

    msg_vel.linear = vel;

    vel_pub_->publish(msg_vel);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;//_ por convención de estilo
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNodePublisher>();

  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    node->pub_vel();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}

