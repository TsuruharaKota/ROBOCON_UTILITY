#include <cmath>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

using std::vector;
using std::atan;
using std::cout;
using std::endl;
using std::abs;

constexpr int division = 5;
struct vector2 {
  double x;
  double y;
  vector2(double _x, double _y) : x(_x), y(_y) {}
};

vector<vector2> map_point = {{0, 0},       {5000, 0},    {5500, 500},
                             {5000, 1000}, {6000, 1000}, {6000, 500},
                             {6000, 0},    {0, 0}};
vector<vector2> goal_map;
vector2 current_point(0, 0);

double calcCircleOf2Podouble(const vector2 point_1, const vector2 point_2,
                             const vector2 point_3, vector2 &real, double &r) {
  long ox, oy, a, b, c, d;
  long r1, r2, r3;
  double stat;

  a = point_2.x - point_1.x;
  b = point_2.y - point_1.y;
  c = point_3.x - point_1.x;
  d = point_3.y - point_1.y;

  stat = -1;

  if ((a && d) || (b && c)) {
    ox = point_1.x +
         (d * (a * a + b * b) - b * (c * c + d * d)) / (a * d - b * c) / 2;
    if (b) {
      oy = (a * (point_1.x + point_2.x - ox - ox) +
            b * (point_1.y + point_2.y)) /
           b / 2;
    } else {
      oy = (c * (point_1.x + point_3.x - ox - ox) +
            d * (point_1.y + point_3.y)) /
           d / 2;
    }
    r1 = sqrt((ox - point_1.x) * (ox - point_1.x) +
              (oy - point_1.y) * (oy - point_1.y));
    r2 = sqrt((ox - point_2.x) * (ox - point_2.x) +
              (oy - point_2.y) * (oy - point_2.y));
    r3 = sqrt((ox - point_3.x) * (ox - point_3.x) +
              (oy - point_3.y) * (oy - point_3.y));
    real.x = ox;
    real.y = oy;
    r = (r1 + r2 + r3) / 3;
    stat = 0;
  }

  return stat;
}

double straightLine_x(const vector2 now, const vector2 prev) {
  for (int n = 1; n <= division; ++n) {
    vector2 cal(0, 0);
    cal.x = now.x;
    cal.y = prev.y + (((now.y - prev.y) / division) * n);
    goal_map.push_back(cal);
  }
}

double straightLine_y(const vector2 now, const vector2 prev) {
  for (int n = 1; n <= division; ++n) {
    vector2 cal(0, 0);
    cal.y = now.y;
    cal.x = prev.x + (((now.x - prev.x) / division) * n);
    goal_map.push_back(cal);
  }
}

double circleCurve(const vector2 prev, const vector2 now, const vector2 after) {
  vector2 middle(0, 0);
  vector2 mid_1(0, 0);
  vector2 mid_2(0, 0);
  vector2 mid_3(0, 0);
  double r;
  calcCircleOf2Podouble(prev, now, after, middle, r);
  double x_s = 0, y_s = 0;
  y_s = middle.x - prev.x;
  x_s = middle.y - prev.y;
  // cout << "x_s " << x_s << endl;
  // cout << "y_s " << y_s << endl;
  double theta_s = atan2(y_s, x_s);
  double x_e = 0, y_e = 0;
  y_e = after.x - middle.x;
  x_e = -(after.y - middle.y);
  double theta_e = atan2(y_e, x_e);
  // cout << "theta_s " << theta_s << endl;
  // cout << "theta_e " << theta_e * (180 / M_PI) << endl;
  double devide_angle = (theta_e - theta_s) / 4;
  // cout << "devide_angle " << devide_angle << endl;
  double theta_1 = theta_s + devide_angle;
  double theta_2 = theta_s + (devide_angle * 2);
  double theta_3 = theta_s + (devide_angle * 3);
  theta_1 > M_PI / 2 ? mid_1.y = r + abs((r * cos(theta_1))) + prev.y
                     : mid_1.y = r - (r * cos(theta_1)) + prev.y;
  mid_1.x = r * sin(theta_1) + middle.x + abs(prev.x - middle.x);
  theta_2 > M_PI / 2 ? mid_2.y = r + abs((r * cos(theta_2))) + prev.y
                     : mid_2.y = r - (r * cos(theta_2)) + prev.y;
  mid_2.x = r * sin(theta_2) + middle.x + abs(prev.x - middle.x);
  theta_3 > M_PI / 3 ? mid_3.y = r + abs((r * cos(theta_3))) + prev.y
                     : mid_3.y = r - (r * cos(theta_3)) + prev.y;
  mid_3.x = r * sin(theta_3) + middle.x + abs((prev.x - middle.x));
  goal_map.push_back(prev);
  goal_map.push_back(mid_1);
  goal_map.push_back(mid_2);
  goal_map.push_back(mid_3);
  goal_map.push_back(after);
  /*cout << "r = " << r << endl;
  cout << "theta_1 " << theta_1 * (180 / M_PI) << endl;
  cout << "theta_2 " << theta_2 * (180 / M_PI) << endl;
  cout << "theta_3 " << theta_3 * (180 / M_PI) << endl;
  cout << "(middle.x, middle.y) : (" << middle.x << ", " << middle.y << ")"
       << endl;
  cout << "(mid_1.x, mid_1.y) : (" << mid_1.x << ", " << mid_1.y << ")" << endl;
  cout << "(mid_2.x, mid_2.y) : (" << mid_2.x << ", " << mid_2.y << ")" << endl;
  cout << "(mid_3.x, mid_3.y) : (" << mid_3.x << ", " << mid_3.y << ")" << endl;
  */
}

double createMap() {
  for (int i = 0; i < 8; ++i) {
    if (i != 0) {
      // x軸上が等しい時
      if (map_point[i].x == map_point[i - 1].x) {
        straightLine_x(map_point[i], map_point[i - 1]);
        // y軸上が等しい時
      } else if (map_point[i].y == map_point[i - 1].y) {
        straightLine_y(map_point[i], map_point[i - 1]);
        // x軸もy軸も等しくない時は円弧補間する
      } else {
        circleCurve(map_point[i - 1], map_point[i], map_point[i + 1]);
        ++i;
      }
    } else {
      goal_map.push_back(map_point[0]);
    }
  }
}

void odomCallback(const geometry_msgs::Point &msg) {
  current_point.x = msg.x;
  current_point.y = msg.y;
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
  static int counter = 0;
  createMap();
  ros::Rate leep_rate(100);
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
