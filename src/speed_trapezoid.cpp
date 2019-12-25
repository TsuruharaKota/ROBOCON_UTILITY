#include <iostream>
#include <ros.h>
#include <std_msgs::Float64>
#include <std_msgs::Float64MultiArray>

using std::cout;
using std::endl;

constexpr double ACCELERATION;
constexpr double GOALSPEED;

double point[2]{};
double total_distance{};

struct Length {
  double start_point;
  double full_length;
  double Vs;
  double Ve;
  double acceleration_length;
  double deceleration_length;
}

void pointCallback(const std_msgs::Float64MultiArray &msg){
  //----------point[0] means start point----------
  //----------point[1] means goal point----------
  point[0] = msg.data[0];
  point[1] = msg.data[1];
}

void distanceCallback(const std_msgs::Float64 &msg) {
  total_distance = msg.data;
}

void lengthCal(Length &get_value) {
  // Vs value should be changed;
  get_value.Vs = 1;
  get_value.Ve = 0;
  get_value.full_length = point[1] - point[0];
  get_value.acceleration_length = pow(GOALSPEED - Vs, 2) / 2;
  get_value.deceleration_length = pow(GOALSPEED - Ve, 2) / 2;
  get_value.start_point = point[0];
}

double velCal(Length &get_value) {
  double vel;
  if (total_distance < get_value.start_point + get_value.acceleration_length) {

  } else if (total_distance < get_value.start_point + get_value.deceleration) {

  } else {
    vel = GOAL_SPEED;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "accel");
  ros::NodeHandle n;
  ros::Subscriber point_sub = n.subscribe("point", 10, pointCallback);
  ros::Subscriber total_distance_sub =
      n.subscribe("total_distance", 10, distanceCallback);
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64>("real_vel", 10);
  ros::Rate loop_rate(100);
  std_msgs::Float64 real_vel;
  Length value;
  static double initial_velocity;

  while (ros::ok()) {
    lengthCal(value);
    real_vel.data = velCal(value);
    initial_velocity = value.Ve;
    velocity_pub.publish(real_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
