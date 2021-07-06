#include <ros.h>
#include <rospy_tutorials/Floats.h>

//MOTOR1
#define RPWM 4
#define LPWM 5
#define LED 13

double output = 0;

ros::NodeHandle nh;

void angleCb(const rospy_tutorials::Floats& cmd_msg) {
  output = cmd_msg.data[0];
}

ros::Subscriber<rospy_tutorials::Floats> sub("/cmd_vel", angleCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
}

void loop() {
  pwmOut(output);
  nh.spinOnce();
}

void pwmOut(float out) {
  if (out > 0) {
    analogWrite(LPWM, out); //cw
    analogWrite(RPWM, 0);
    digitalWrite(LED, HIGH);
  }
  else {
    analogWrite(LPWM, 0); //ccw
    analogWrite(RPWM, abs(out));
    digitalWrite(LED, LOW);
  }
}
