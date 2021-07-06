#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>
#include <ArduinoHardware.h>

#define LOOPTIME 100
#define encoA 3
#define encoB 2
#define RPWM 5
#define LPWM 4

float pos_motor = 0;
unsigned long lastTime = 0;
const double radius = 0.07;

double speed_req_motor = 0;
double speed_act_motor = 0;
double speed_cmd_motor = 0;
int PWM_value = 0;

ros::NodeHandle nh;

void cb_cmd(const geometry_msgs::Twist& cmd_vel) {
  PWM_value = cmd_vel.linear.x*100;
  analogWrite(RPWM, PWM_value);
}

geometry_msgs::Vector3Stamped speed_msg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel("/cmd_vel", &cb_cmd);
ros::Publisher speed_pub("speed", &speed_msg);

void setup() {
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(speed_pub);

  pinMode (RPWM, OUTPUT);
  pinMode (LPWM, OUTPUT);
  pinMode(encoA, INPUT_PULLUP);
  pinMode(encoB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoA), readEncoder, RISING);
}

void loop() {
  if ((millis() - lastTime) >= LOOPTIME) {
    lastTime = millis();
    speed_act_motor = ((pos_motor / 448) * PI) * (1000 / LOOPTIME) * radius;
    pos_motor = 0;

    publishSpeed();
  }
  nh.spinOnce();
}

void publishSpeed() {
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = speed_act_motor;
  speed_msg.vector.z = PWM_value;
  speed_pub.publish(&speed_msg);

  nh.spinOnce();
  nh.loginfo("Publishing SPEED for PWM");
}

void readEncoder() {
  int b = digitalRead(encoB);

  if (pos_motor > 2250 || pos_motor < -2250) {
    pos_motor = 0;
  }
  if (b > 0) {
    pos_motor++;
  }
  else {
    pos_motor--;
  }
}
