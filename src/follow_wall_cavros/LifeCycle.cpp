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

#define ANGULAR_VEL 0.2
#define LINEAR_VEL 0.2

LifeCycle::LifeCycle(const std::string & name, const std::chrono::nanoseconds & rate)
: LifecycleNode(name)
{
  // Velocity publisher and laser subscriber
  pub_ = create_publisher<geometry_msgs::msg::Twist>("key_vel", 10);
  sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "laser_info", 10, std::bind(&LifeCycle::distance_callback, this, _1));

  timer_ = create_wall_timer(rate, std::bind(&LifeCycle::do_work, this));
}

// -- TRANSICIONES --
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT LifeCycle::on_configure(const rclcpp_lifecycle::State & state)
{
  // Se establecen los parametros
  RCLCPP_INFO(get_logger(), "[%s] On_configure desde [%s]", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LifeCycle::on_activate(const rclcpp_lifecycle::State & state)
{
  // Crear timer + activar publicador (velocidades)
  RCLCPP_INFO(get_logger(), "[%s] On_activate desde [%s]", get_name(), state.label().c_str());

  // Activate speed publisher
  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT LifeCycle::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Destruir timer + desactivar publicador
  RCLCPP_INFO(get_logger(), "[%s] On_deactivate desde [%s]", get_name(), state.label().c_str());

  // Deactivate spped publisher
  pub_.reset();
  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

void LifeCycle::do_work(void)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (pub_->is_activated()) {
    
    choose_speeds();
    publish_vel();
  }
}

void LifeCycle::choose_speeds(void)
{
  // aproach to wall
  if (min_distance_ > 0.7) {
    RCLCPP_INFO(this->get_logger(), "[%s] APROACHING WALL", get_name());

    //linear
    if (fabs(angle_) < 10) {
      x_ = LINEAR_VEL * 1.5;
    } else if (fabs(angle_) < 20) {
      x_ = LINEAR_VEL;
    } else {
      x_ = LINEAR_VEL / 2;
    }
    
    //angular
    if (angle_ > 10) {
      z_ = ANGULAR_VEL; // rotate left to face wall
    } else if (angle_ < -10) {
      z_ = -ANGULAR_VEL; // rotate right to face wall
    } else {
      z_ = 0.0; // aproach wall
    }

  } else {  // already close to wall
    RCLCPP_INFO(this->get_logger(), "[%s] FOLLOWING WALL", get_name());

    // lineal
    if (angle_ > -105 && angle_ < -75) {  // parallel to wall
      x_ = LINEAR_VEL + 0.1;
    } else if (min_distance_ < 0.3) {  // too close to wall go back
      x_ = -LINEAR_VEL/4;
    } else {
      x_ = 0.0;
    }

    // angular
    if (angle_ < -95) {  // turn right
      z_ = -ANGULAR_VEL;
    } else if(angle_ > -85) {  // turn left
      z_ = ANGULAR_VEL;
    } else {
      z_ = 0.0;
    }

    if (right_distance_ > 2.5 && angle_ < 0) {  // detect open door
      
      if (min_distance_ < 0.5) {
        x_ = 0.42;
        z_ = -0.5;
      } else {
        x_ = 0.32;
        z_ = -0.3;
      }
    }
  }
}

void LifeCycle::publish_vel(void)
{
  geometry_msgs::msg::Vector3 vel_linear, vel_angular;
  geometry_msgs::msg::Twist msg_vel;

  vel_linear.x = x_;
  vel_angular.z = z_;
  msg_vel.linear = vel_linear;
  msg_vel.angular = vel_angular;

  pub_->publish(msg_vel);
}

void LifeCycle::distance_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  angle_ = msg->data[0];
  min_distance_ = msg->data[1];
  right_distance_ = msg->data[2];
} 
