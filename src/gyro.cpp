#include <cmath>
#include <iostream>
#include <ros.h>
#include <std_msgs/Float64.h>

using std::cout;
using std::endl;
using std_msgs::Float64 gyro;

int main(int argc, char **argv) {
  ros::init(argc, argv, "gyro");
  ros::NodeHandle n;
  ros::Publisher gyro_pub = n.advertise<std_msgs::Float64>("gyro", 10);

  ros::Rate loop_rate(500);

  while (ros::ok()) {
    gyro_pub.publish(gyro);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
