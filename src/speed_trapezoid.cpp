#include <iostream>
#include <ros.h>
#include <std_msgs::Float64>
#include <std_msgs::Float64MultiArray>

using std::cout;
using std::endl;

constexpr double ACCELERATION = 0.5;
constexpr double ACCELERATION_SUB = 0.7;
constexpr double GOALSPEED = 1.5;

double point[2]{};
double total_distance{};
double velocity_start = 1.0;
double velocity_end;

struct Length {
  double start_point;
  double full_length;
  double Vs;
  double Ve;
  double acceleration_length;
  double deceleration_length;
  double middle_length;
}

void pointCallback(const geometry_msgs::Point &msg){
  //----------point[0] means start point----------
  //----------point[1] means goal point----------
  point[0] = msg.x;
  point[1] = msg.y;
}

void distanceCallback(const std_msgs::Float64 &msg) {
  total_distance = msg.data;
}

void velocityCallback(const std_msgs::Float64 &msg) {
  velocity_final = msg.data;
}

void lengthCal(Length &get_value, bool &flag) {
  // Vs value should be changed;
  get_value.Vs = velocity_start;
  get_value.Ve = velocity_final;
  get_value.full_length = point[1] - point[0];
  get_value.acceleration_length = pow(GOALSPEED - Vs, 2) / 2;
  get_value.deceleration_length = pow(GOALSPEED - Ve, 2) / 2;
  get_value.middle_length =
      full_length - get_value.acceleration - get_value.deceleration;
  get_value.start_point = point[0];
  get_value.middle_length == 0 ? flag = true : flag = false;
}

double velCal(Length &get_value, bool flag) {
  double vel;
  if (total_distance < get_value.start_point + get_value.acceleration_length) {
    flag ? vel = ACCELERATION * (total_distance - point[0]) + Vs
         : ACCELERATION_SUB * (total_distance - point[0]) + Vs;
  } else if (total_distance > get_value.start_point + get_value.middle_length) {
    flag ? vel = -ACCELERATION * (total_distance - point[0]) + Vs
         : -ACCELERATION_SUB * (total_distance - point[0]) + Vs;
  } else {
    vel = GOAL_SPEED;
  }
  return vel;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "accel");
  ros::NodeHandle n;
  ros::Subscriber point_sub = n.subscribe("point", 10, pointCallback);
  ros::Subscriber total_distance_sub =
      n.subscribe("total_distance", 10, distanceCallback);
  ros::Subscriber velocity_sub =
      n.subscribe("velocity_end", 10, velocityCallback);
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64>("real_vel", 10);
  ros::Rate loop_rate(100);
  std_msgs::Float64 real_vel;
  Length value;
  bool flag_change_accel = false;
  //最終目標速度(Ve)を受け取るように再実装する必要がある
  while (ros::ok()) {
    lengthCal(value, flag_change_accel);
    real_vel.data = velCal(value, flag_change_accel);
    velocity_start = value.Ve;
    velocity_pub.publish(real_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
