#include <iostream>
#include <ros.h>
#include <std_msgs::Float64>
#include <std_msgs::Float64MultiArray>

using std::cout;
using std::endl;

constexpr double ACCELERATION;
constexpr double GOALSPEED;
double point[2]{};

struct Length {
  double full_length;
  double acceleration_length;
  double deceleration_length;
}

void pointCallback(const std_msgs::Float64MultiArray &msg){
  //----------point[0] means start point----------
  //----------point[1] means goal point----------
  point[0] = msg.data[0];
  point[1] = msg.data[1];
}

void lengthCal(Length &get_value) {
  get_value.full_length = point[1] - point[0];
  get_value.acceleration_length = ;
  get_value.deceleration_length = point[1] - point[0];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "accel");
  ros::NodeHandle n;
  ros::Subscriber point_sub = n.subscribe("point", 10, pointCallback);
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64>("real_vel", 10);
  ros::Rate loop_rate(100);
  Length value;
  static double initial_velocity;

  while (ros::ok()) {
    lengthCal(value);
    initial_velocity = value.ros::spinOnce();
    loop_rate.sleep();
  }
}
