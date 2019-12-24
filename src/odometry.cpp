#include <cmath>
#include <geometry_msgs::Point.h>
#include <iostream>
#include <pigpiod.hpp>
#include <ros.h>
#include <rotary_inc.hpp>
#include <vector>
// publish current odometry
int main(int argc, char **argv) {
  ros::init(argc, argv, "four_omuni_odometry");
  ros::NodeHandle n;
  ros::Publisher odom_pub =
      n.advertise<geometry_msgs::Point>("odometry_point", 50);
  geometry_msgs::Point odometry_point;
  constexpr double WHEEL_CIRC = 101.6 * M_PI;
  constexpr int RANGE = 200;
  vector<double> wheel_pos{0, 0, 0, 0};
  RotaryInc rotary[4] = {RotaryInc(1, 2, 3), RotaryInc(4, 5, 6),
                         RotaryInc(7, 8, 9), RotaryInc(10, 11, 12)};
  ros::Rate loop_rate(200);

  while (ros::ok()) {

    wheel_pos[0] += (static_cast<int>(rotary[0].diff()) / RANGE) * WHEEL_CIRC;
    wheel_pos[1] += (static_cast<int>(rotary[1].diff()) / RANGE) * WHEEL_CIRC;
    wheel_pos[2] += (static_cast<int>(rotary[2].diff()) / RANGE) * WHEEL_CIRC;
    wheel_pos[3] += (static_cast<int>(rotary[3].diff()) / RANGE) * WHEEL_CIRC;

    odometry_point.x = (-wheel_pos[1] + wheel_pos[3]) / 2;
    odometry_point.y = (wheel_pos[0] - wheel_pos[2]) / 2;

    odom_pub.publish(odometry_point);

    loop_rate.sleep();
  }
  return 0;
}
