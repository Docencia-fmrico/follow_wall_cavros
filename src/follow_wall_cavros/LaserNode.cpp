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
    //data in msg->ranges goes from right to left
    //there are 665 values

    float min = 25, angle;
    std_msgs::msg::Float32MultiArray info;
    int pos,j = 0;

    //angle range follows:
    //            ^ 0º
    //            | 
    // 90º <--  robot  --> -90º ( ranges[0] = -108º ; and last ranges[665] = 108º)  each angle has 3.055 values

    for(int i=0 ; i < 666 ; i++){
      //min distance
      if(msg->ranges[i] < min){
        min = msg->ranges[i];
        pos = i;
      }
    }

    angle = ( pos - 329.94 ) / 3.055;

    info.data.push_back(angle);//angle respect to min distance
    info.data.push_back(min);
    info.data.push_back(msg->ranges[131]);//laser value to detect open doors

    laser_info_pub_->publish(info);

  }

}//namespace LaserNode