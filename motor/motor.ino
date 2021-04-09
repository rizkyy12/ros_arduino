#include <ros.h>
#include <geometry_msgs/Twist.h>

//MOTOR1
#define RPWM1 4
#define LPWM1 5

float Vm_x, Vm_y, Vm_z;
float V1, V2, V3;

ros::NodeHandle nh;

void motorCb(const geometry_msgs::Twist& msg){
  Vm_x = msg.linear.x;
  Vm_y = msg.linear.y;
  Vm_z = msg.linear.z;

  V1 = (-Vm_x / 2) - (sqrt(3)*Vm_y/2) + Vm_z;
  
  if (V1 >= 0){
    digitalWrite(RPWM1, V1);
    digitalWrite(LPWM1, 0);
  }
  else {
    digitalWrite(RPWM1, 0);
    digitalWrite(LPWM1, -V1);
  }
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motorCb);

void setup() {

  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
