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

#include "follow_wall-cavros/LaserNode.hpp"

TEST(test_node, velocity)
{
}

TEST(test_node, distance)
{
  auto node = std::make_shared<follow_wall-cavros::LaserNode>();

  ASSERT_EQ("", node->get_last_msg());
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
