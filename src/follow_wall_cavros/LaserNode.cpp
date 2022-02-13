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

    float min = 25, angle;
    std_msgs::msg::Float32MultiArray info;
    int pos;

    for(int i=0 ; i < 666 ; i++){
      if(msg->ranges[i] < min){
        min = msg->ranges[i];
        pos = i;
      }
    }
    //angle range follows:
    //            ^ 0º
    //            | 
    // 90º <--  robot  --> -90º ( first angle = -108º ; and last = 108º)

    angle = ( pos - 54 ) / 3;
    RCLCPP_INFO(this->get_logger(),"Data laser= %f\n",msg->ranges[333]);

    info.data.push_back(min);
    info.data.push_back(angle);
    RCLCPP_INFO(this->get_logger(),"min distance:%f \n",info.data[0]);
    RCLCPP_INFO(this->get_logger(),"angle:%f \n",info.data[1]);


    laser_info_pub_->publish(info);

  }

}//namespace LaserNode