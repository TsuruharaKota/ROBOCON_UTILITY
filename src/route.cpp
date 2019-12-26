#include <iostream>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

double current_x, current_y;

double map_point[][2]{{1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5},
                      {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}};

void odomCallback(const geometry_msgs::Point &msg) {}

bool check(double goal_point) {
  if (goal_point.x - 0.1 < current_x && current_x < goal_point.x + 0.1) {
  }
  if (goal_point.y - 0.1 < current_y && current_y < goal_point.y + 0.1)
}

double straightLine(const double t) {}

double bezierCurve(const double t) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "route");
  ros::NodeHandle n;
  ros::Publisher route_pub =
      n.advertise<std_msgs::Float64MultiArray>("point", 10);
  ros::Subscriber odom_sub = n.subscribe("odometry_point", 10, odomCallback);
  std_msgs::Float64MultiArray route;
  route.data.resize(2);
  ros::Rate leep_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
