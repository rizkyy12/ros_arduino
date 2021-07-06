#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ArduinoHardware.h>

//MOTOR1
#define RPWM1 11
#define LPWM1 10

//MOTOR2
#define RPWM2 9
#define LPWM2 6

//MOTOR3
#define RPWM3 5
#define LPWM3 3

float Vm_x, Vm_y, Vm_z;
float V1, V2, V3;

void motorCb(const geometry_msgs::Twist& msg){
  Vm_x = msg.linear.x;
  Vm_y = msg.linear.y;
  Vm_z = msg.linear.z;

  V1 = (-Vm_x / 2) - (sqrt(3)*Vm_y/2) + Vm_z;
  V2 = Vm_x + Vm_z;
  V3 = (-Vm_x / 2) + (sqrt(3)*Vm_y/2) + Vm_z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motorCb);
ros::NodeHandle nh;

void setup() {
  // put your setup code here, to run once:
  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);

  pinMode (RPWM2, OUTPUT);
  pinMode (LPWM2, OUTPUT);

  pinMode (RPWM3, OUTPUT);
  pinMode (LPWM3, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  if (V1 >= 0){
    analogWrite(RPWM1, V1);
    analogWrite(LPWM1, 0);
  }
  else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -V1);
  }
  if (V2 >= 0){
    analogWrite(RPWM2, V2);
    analogWrite(LPWM2, 0);
  }
  else {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, -V2);
  }
  if (V3 >= 0){
    analogWrite(RPWM3, V3);
    analogWrite(LWPM3, 0);
  }
  else {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, -V3);
  }
  nh.spinOnce();
  delay(1);
}
