#include "follow_wall_cavros/LaserNode.hpp"
using namespace std::chrono_literals;//500ms...



namespace follow_wall_cavros{ 

    LaserNode::LaserNode(const std::string & name)
    : Node(name)
    {
        sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>("scan_raw", 10, std::bind(&LaserNode::Laser_callback, this, _1));
    }
  void LaserNode::Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) /*const*/{
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