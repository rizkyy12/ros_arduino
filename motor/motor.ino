#include <ros.h>
#include <geometry_msgs/Twist.h>

//MOTOR1
#define RPWM1 5
#define LPWM1 4
#define LED 13

float Vm_x, Vm_y, Vm_z;
float V1, V2, V3;

ros::NodeHandle nh;

void motorCb(const geometry_msgs::Twist& msg){  
  Vm_x = msg.linear.x;
  Vm_y = msg.linear.y;
  Vm_z = msg.linear.z;

  V1 = (-Vm_x / 2) - (sqrt(3)*Vm_y/2) + Vm_z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motorCb);

void setup() {
  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  if (V1 >= 0){
    analogWrite(RPWM1, V1);
    analogWrite(LPWM1, 0);
    digitalWrite(LED, HIGH);
  }
  else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -V1);
    digitalWrite(LED, LOW);
  }
  nh.spinOnce();
  delay(1);
}
