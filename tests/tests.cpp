// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "follow_wall_cavros/LaserNode.hpp"

/*
// Test for velocity check (Motors)
TEST(test_node, velocity)
{
  auto node = std::make_shared<follow_wall_cavros::MoveNode>("test_move_node");

  auto test_node = rclcpp::Node::make_shared("test_pub_move_node");
  auto vel_pub = test_node->create_publisher<std_msgs::msg::Float32MultiArray>("laser_info", 10);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  exe.add_node(test_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  std_msgs::msg::Float32MultiArray msg;

  msg.data.push_back(45);
  msg.data.push_back(0.5);
  msg.data.push_back(100);

  vel_pub->publish(msg);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(0.3, node->get_linear());
  ASSERT_EQ(0.2, node->get_angular());

  msg.data.push_back(0);
  msg.data.push_back(1);
  msg.data.push_back(100);

  vel_pub->publish(msg);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(0.35, node->get_linear());
  ASSERT_EQ(0.0, node->get_angular());

  finish = true;
  t.join();
}

// Test for distances check (Laser)
*/
TEST(test_node, distance)
{
  auto node = std::make_shared<follow_wall_cavros::LaserNode>("test_laser_node", 500ms);

  auto test_node = rclcpp::Node::make_shared("test_pub_laser_node");
  auto laser_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>("laser_info", 10);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  exe.add_node(test_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  sensor_msgs::msg::LaserScan laser;

  for (int i = 0; i < 666; i++) {
    laser.ranges[i] = 2;
  }
  laser.ranges[330] = 1;

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(0, (int)node->get_angle());
  ASSERT_EQ(1, node->get_min_distance());
  ASSERT_EQ(false, node->door_open());


  for (int i = 0; i < 666; i++) {
    laser.ranges[i] = 2;
  }
  laser.ranges[54] = 1.5;
  laser.ranges[313] = 3;

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(-90, (int)node->get_angle());
  ASSERT_EQ(1, node->get_min_distance());
  ASSERT_EQ(false, node->door_open());

  finish = true;
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
