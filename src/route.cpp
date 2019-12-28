#include <geometry_msgs/Point.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

double map_point[][2]{{1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5},
                      {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}};
struct vector2 {
  double x;
  double y;
  vector2(double _x, double _y) : x(_x), y(_y) {}
};

vector2 current_point(0, 0);
vector<vector2> goal_map;

void odomCallback(const geometry_msgs::Point &msg) {
  current_point.x = msg.x;
  current_point.y = msg.y;
}

double createMap() {}
double straightLine(const double t) {}
double circleCurve(const double t) {}

bool check(vector2 goal_point) {
  if (goal_point.x - 0.1 < current_point.x &&
      current_point.x < goal_point.x + 0.1) {
    if (goal_point.y - 0.1 < current_point.y &&
        current_point.y < goal_point.y + 0.1)
      return true;
  }
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "route");
  ros::NodeHandle n;
  ros::Publisher route_pub = n.advertise<geometry_msgs::Point>("point", 10);
  ros::Publisher velocity_pub =
      n.advertise<std_msgs::Float64>("velocity_end", 10);
  ros::Subscriber odom_sub = n.subscribe("odometry_point", 10, odomCallback);
  geometry_msgs::Point route;
  std_msgs::Float62 real_Ve;
  route.data.resize(2);
  createMap();
  ros::Rate leep_rate(100);
  static int counter = 0;
  vector<double> Ve_param = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                             1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  while (ros::ok()) {
    //値の更新、checkでオドメトリが目標地点内に入っていたら更新する
    if (check(goal_map[counter - 1])) {
      route.x = goal_map[counter].x;
      route.y = goal_map[counter].y;
      real_Ve.data = Ve_param[counter];
      ++counter;
    }
    route_pub.publish(route);
    velocity_pub.publish(real_Ve);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
