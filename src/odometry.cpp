#include <cmath>
#include <geometry_msgs::Point.h>
#include <iostream>
#include <pigpiod.hpp>
#include <ros.h>
#include <rotary_inc.hpp>
#include <std_msgs::Float64.h>
#include <vector>
// publish current odometry
int main(int argc, char **argv) {
  ros::init(argc, argv, "four_omuni_odometry");
  ros::NodeHandle n;
  ros::Publisher odom_pub =
      n.advertise<geometry_msgs::Point>("odometry_point", 50);
  ros::Publisher total_distance_pub =
      n.advertise<std_msgs::Float64>("total_distance", 50);
  geometry_msgs::Point odometry_point;
  std_msgs::Float64 total_distance;
  constexpr double WHEEL_CIRC = 101.6 / 2;
  constexpr int RANGE = 200;
  constexpr int FREQUENCY = 200;
  vector<double> wheel_vel{0, 0, 0, 0};
  vector<double> wheel_dis{0, 0};
  RotaryInc rotary[4] = {RotaryInc(1, 2, 3), RotaryInc(4, 5, 6),
                         RotaryInc(7, 8, 9), RotaryInc(10, 11, 12)};
  ros::Rate loop_rate(200);

  while (ros::ok()) {

    wheel_vel[0] =
        (((static_cast<double>(rotary[0].diff()) / RANGE) * 2 * M_PI) /
         FREQUENCY) *
        WHEEL_CIRC;
    wheel_vel[1] =
        (((static_cast<double>(rotary[1].diff()) / RANGE) * 2 * M_PI) /
         FREQUENCY) *
        WHEEL_CIRC;
    wheel_vel[2] =
        (((static_cast<double>(rotary[2].diff()) / RANGE) * 2 * M_PI) /
         FREQUENCY) *
        WHEEL_CIRC;
    wheel_vel[3] =
        (((static_cast<double>(rotary[3].diff()) / RANGE) * 2 * M_PI) /
         FREQUENCY) *
        WHEEL_CIRC;

    odometry_point.x += (-wheel_vel[1] + wheel_vel[3]) * FREQUENCY;
    odometry_point.y += (wheel_vel[0] - wheel_vel[2]) * FREQUENCY;
    odometry_dis[0] += abs(-wheel_vel[1] + wheel_vel[3]);
    odometry_dis[1] += abs(wheel_vel[0] - (wheel_vel[2]);
    total_distance.data =
        sprt(pow(odometry_dis[0], 2) * pow(odometry_dis[1], 2));

    odom_pub.publish(odometry_point);
    total_distance_pub.publish(total_distance);

    loop_rate.sleep();
  }
  return 0;
}
