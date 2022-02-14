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

LifeCycle::LifeCycle() 
: LifecycleNode("LifeCycle")
{
    pub_node_ = std::make_shared<follow_wall_cavros::MoveNode>("Move_node");
    sub_node_ = std::make_shared<follow_wall_cavros::LaserNode>("Laser_Node");
}

// -- TRANSICIONES -- 
using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT LifeCycle::on_configure(const rclcpp_lifecycle::State & state)
{
    // Se establecen los parametros
    RCLCPP_INFO(get_logger(), "[%s] On_configure desde [%s]", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS; //FAILURE
}

CallbackReturnT LifeCycle::on_activate(const rclcpp_lifecycle::State & state)
{
    // Crear timer + activar publicador (velocidades)
    RCLCPP_INFO(get_logger(), "[%s] On_activate desde [%s]", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT LifeCycle::on_deactivate(const rclcpp_lifecycle::State & state)
{
    // Destruir timer + desactivar publicador 
    RCLCPP_INFO(get_logger(), "[%s] On_deactivate desde [%s]", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
}

void 
LifeCycle::do_work()
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return;
    }

    RCLCPP_INFO(get_logger(), "[%s] Ejecutando do_work...", get_name());
    pub_node_->pub_vel();
    rclcpp::spin_some(pub_node_);
    rclcpp::spin_some(sub_node_);
}