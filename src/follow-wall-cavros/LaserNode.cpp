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

#include "follow-wall-cavros/LaserNode.hpp"

using namespace std::chrono_literals;//500ms...

namespace LaserNode{

  void LaserNode::pub_vel(void)
  {
    geometry_msgs::msg::Vector3 vel;
    geometry_msgs::msg::Twist msg_vel;

    vel.x = -3.0;
    vel.y = 0.0;
    vel.z = 0.0;

    msg_vel.linear = vel;

    vel_pub_->publish(msg_vel);
  }

  void LaserNode::Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const{
    //Laser Range of distances : min = 0.05m ; max = 25m 
    //los datos en msg->ranges van de derecha a izquierda
    //hay 665 valores

    float min = 25;
    int pos, angle ;

    for(int i=0 ; i < 666 ; i++){
      if(msg->ranges[i] < min){
        min = msg->ranges[i];
        pos = i;
      }
    }
    
    //angle range follows:
    //            ^ 90º
    //            | 
    // 180º <--  robot  --> 0º ( minimum angle = -18º ; and maximum = 198º)

    angle = ( pos - 54 ) / 3;
    RCLCPP_INFO(this->get_logger(),"min distance:%f at angle= %d\n",min,angle);

  }

}//namespace LaserNode

int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LaserNode::LaserNode>("Laser_Node");

    rclcpp::Rate loop_rate(500ms);
    while (rclcpp::ok()) {
      node->pub_vel();

      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
  }
  