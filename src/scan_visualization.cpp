#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

using namespace std;

ros::Publisher my_scan_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    sensor_msgs::LaserScan my_msg;
    my_msg = *scan_msg;
    cout << "La cantidad de puntos que entrega el lidar son: " << my_msg.ranges.size() << endl;
    for(int i = 0; i < my_msg.ranges.size(); i++){
        if(!(i >= 90 && i <= 270)){
            my_msg.ranges[i] = 0;
            
        }
    }
    my_scan_pub.publish(my_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  
  my_scan_pub = n.advertise<sensor_msgs::LaserScan>("/my_scan", 1000);

  ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
    
  ros::spin();
  
  return 0;
}
