#include "follow_wall_cavros/LaserNode.hpp"

using namespace std::chrono_literals;//500ms...

namespace follow_wall_cavros{ 

    LaserNode::LaserNode(const std::string & name)
    : Node(name)
    {
        sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>("scan_raw", 10, std::bind(&LaserNode::Laser_callback, this, _1));
        laser_info_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("laser_info", 10);
    }

  void LaserNode::Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const{
    //Laser Range of distances : min = 0.05m ; max = 25m 
    //los datos en msg->ranges van de derecha a izquierda
    //hay 665 valores

    float min = 25, angle ,sudden_change_angles[4] = {0,0,0,0};
    std_msgs::msg::Float32MultiArray info;
    int pos,j = 0;

    //angle range follows:
    //            ^ 0º
    //            | 
    // 90º <--  robot  --> -90º ( ranges[0] = -108º ; and last = 108º) [ angle*3.005 = pos ]

    for(int i=0 ; i < 666 ; i++){
      //min distance
      if(msg->ranges[i] < min){
        min = msg->ranges[i];
        pos = i;
      }

    }

    angle = ( pos - 329.94 ) / 3.055;

    info.data.push_back(angle);
    info.data.push_back(min);
    info.data.push_back(msg->ranges[131]);
  /*
    for ( int i = 0; i< 4; i++){
      if(sudden_change_angles[i] != 0){
        info.data.push_back(sudden_change_angles[i]);
      }
    }
    */

    laser_info_pub_->publish(info);

  }

}//namespace LaserNode