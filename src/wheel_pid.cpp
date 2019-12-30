#include <cmath>
#include <iostream>
#include <ros.h>
#include <std_msgs::Float64.h>

class PID {
public:
  PID(const double *gain, const double freq, const int resolution);
  void PidUpdate(double goal, double now, double prev);
  double Get();
  void SetGain(const double *gain);

private:
  void Defferential();
  void Integral();
  void Calcurate();
  double Kp;
  double Ki;
  double Kd;
  double FREQ;
  double goal_value = 0;
  double now_value = 0;
  double prev_value = 0;
  double answer_value = 0;
  double integral_value = 0;
  double defferential_value = 0;
};

PID::PID(const double *gain, const double freq, const int resolution) {
  Kp = gain[0];
  Ki = gain[1];
  Kd = gain[2];
  FREQ = freq;
}

void PID::SetGain(const double *gain) {
  Kp = gain[0];
  Ki = gain[1];
  Kd = gain[2];
}

void PID::PidUpdate(double goal, double now, double prev) {
  goal_value = goal;
  now_value = now;
  prev_value = prev;

  Defferential();
  Integral();
  Calcurate();
}

void PID::Defferential() {
  defferential_value = (now_value - prev_value) / (1 / FREQ);
}

void PID::Integral() { integral_value += now_value * (1 / FREQ); }

void PID::Calcurate() {
  //	cout << goal_value << ":" << now_value << endl;
  answer_value = Kp * (goal_value - now_value);
  -Kd *defferential_value;
}

double PID::Get() { return answer_value; }

void velocityCallback(const std_msgs::Float64 &msg) {
  robot_velocity = msg.data;
}

// Vx, Vyを定義する必要あり
void wheelCal(double &wheel_pwm) {
  wheel_pwm[0] = -Vx * cos(M_PI / 4) + Vy * sin(M_PI / 4);
  wheel_pwm[1] = -Vx * cos(M_PI / 4) - Vy * sin(M_PI / 4);
  wheel_pwm[2] = Vx * cos(M_PI / 4) - Vy * sin(M_PI / 4);
  wheel_pwm[3] = Vx * cos(M_PI / 4) + Vy * sin(M_PI / 4);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_pid");
  ros::NodeHandle n;
  ros::Publisher pwm_pub = n.advertise<std_msgs::Int16>("wheel_pwm", 10);
  ros::Subscriber velocity_sub = n.subscribe("real_vel", 10, velocityCallback);
  ros::Rate loop_rate(100);
  vector<double> wheel_pwm{0, 0, 0, 0};

  while (ros::ok()) {
    wheelCal(&wheel_pwm);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
