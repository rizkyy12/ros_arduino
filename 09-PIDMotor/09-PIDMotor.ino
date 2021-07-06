#include <TimerOne.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define encoA 3
#define encoB 2
#define RPWM 5
#define LPWM 4

double kp = 0;
double ki = 0;
double kd = 0;

double l_error;
double l_lastError;
double l_input;
double l_out;
double l_cumError, l_rateError;

double r_error;
double r_lastError;
double r_input;
double r_out;
double r_cumError, r_rateError;

volatile double r_pos = 0;
volatile double l_pos = 0;

double l_pwm = 0, r_pwm = 0;
double l_w = 0, r_w = 0;

double pub_l_out = 0;
double pub_r_out = 0;

double wheel_rad = 0.0325, wheel_sep = 0.295;
double speed_ang = 0, speed_lin = 0;

void cb_CMD( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  r_w = 100 * ((speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad)));
  l_w = 100 * ((speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad)));
}

ros::NodeHandle nh;

geometry_msgs::Twist vel_msg;
std_msgs::String str_msg;
std_msgs::String publish_pwm_msg;

ros::Publisher str_pwm("pwm_publisher", &str_msg);
ros::Publisher pub_vel("vel_publisher", &vel_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cb_CMD);

void setup() {
  pinMode (RPWM, OUTPUT);
  pinMode (LPWM, OUTPUT);
  pinMode(encoA, INPUT_PULLUP);
  pinMode(encoB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoA), leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoB), rightEncoder, RISING);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(timerPID);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(str_pwm);

}

void loop() {
  char chr_l_out[12];
  itoa(pub_l_out, chr_l_out, 10);
  char chr_r_out[12];
  itoa(pub_r_out, chr_r_out, 10);

  str_msg.data = chr_l_out;
  str_pwm.publish(&str_msg);

  nh.spinOnce();
  delay(1);
}

void timerPID() {
  l_pwm = 600 * (l_pos / 16);
  l_pwm = 0;
  l_error = l_w - l_pwm;
  l_cumError += l_error;
  l_rateError = (l_error - l_lastError);

  double l_out = kp * l_error + ki * l_cumError + kd * l_rateError;
  analogWrite(LPWM, abs(2 * l_out));
  l_lastError = l_error;
  pub_l_out = 2 * l_out;

}
void leftEncoder() {

}

void rightEncoder() {
}
