#include <geometry_msgs/Point.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

constexpr double R = 3.0;
double map_point[10][2]{{1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5},
                        {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}};
struct vector2 {
  double x;
  double y;
  vector2(double _x, double _y) : x(_x), y(_y) {}
};
vector<vector2> map_point = {{1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5},
                             {6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}};
vector<vector2> goal_map;
vector2 current_point(0, 0);

void odomCallback(const geometry_msgs::Point &msg) {
  current_point.x = msg.x;
  current_point.y = msg.y;
}

double createMap() {
  for (int i = 0; i < 10; ++i) {
    if (i != 0) {
      // x軸上が等しい時
      if (map_point[i].x == map_point[i - 1].y) {
        straightLine_x(map_point[i], map_point[i - 1]);
        // y軸上が等しい時
      } else if (map_point[i][1] == map_point[i - 1][1]) {
        straightLine_y(map_point[i], map_point[i - 1]);
        // x軸もy軸も等しくない時は円弧補間する
      } else {
        circleCurve(map_point[i], map_point[i - 1]);
      }
    } else {
      goal_map.push_back(map_point[0]);
    }
  }
}
double straightLine_x(const vector2 now, const vector2 prev) {
  goal_map.push_back(now);
  vector2 cal(0, 0);
  cal.x = (now.x + prev.x) / 2;
  cal.y = now.y;
  goal_map.push_back(cal);
}
double straightLine_y(const vector2 now, const vector2 prev) {
  goal_map.push_back(now);
  vector2 cal(0, 0);
  cal.x = now.x;
  cal.y = (now.y + prev.y) / 2;
  goal_map.push_back(cal);
}
double circleCurve(const vector2 now, const vector2 prev) {
  vector2 middle(0, 0);
  middle.x = middle.y = double theta_s =
      atan((prev.x - middle.x) / (prev.y - middle.y));
  double theta_e = atan((now.x - middle.x) / (now.y - middle.y));
}

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
