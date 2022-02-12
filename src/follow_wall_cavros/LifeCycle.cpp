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
: LifecycleNode("lifecycle_node_example")
{
    // Para declarar parámetros p.ej
}

// -- TRANSICIONES -- 
using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT 
LifeCycle::on_configure(const rclcpp_lifecycle::State & state)
{
    // Se establecen los parametros
    return CallbackReturnT::SUCCESS; //FAILURE
}

CallbackReturnT 
LifeCycle::on_activate(const rclcpp_lifecycle::State & state)
{
    // Crear timer + activar publicador (velocidades)
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
LifeCycle::on_deactivate(const rclcpp_lifecycle::State & state)
{
    // Destruir timer + desactivar publicador 
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
LifeCycle::on_cleanup(const rclcpp_lifecycle::State & state)
{
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
LifeCycle::on_shutdown(const rclcpp_lifecycle::State & state)
{
    return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
LifeCycle::on_error(const rclcpp_lifecycle::State & state)
{
    return CallbackReturnT::SUCCESS;
}

void 
LifeCycle::do_work()
{

}