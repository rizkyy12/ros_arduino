#include <ros.h>
#include <std_msgs/Float64.h>
#include <ArduinoHardware.h>
#include <config.h>

float Vm_x, Vm_y, Vm_z;
float V1, V2, V3;
float Vmx, Vmy, Wp;

int L = 13.5;

ros::NodeHandle nh;

//publish to each motor motor1_Cb, motor2_Cb, motor3_Cb
void motor1_Cb(const std_msgs::Float64 &motor1) {
  V1 = motor1.data;

  if (V1 > 0.0001) {
    analogWrite(RPWM1, V1);
    analogWrite(LPWM1, 0);
  }
  else if (V1 < -0.0001) {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -V1);
  }
  else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
  }
}

void motor2_Cb(const std_msgs::Float64 &motor2) {
  V2 = motor2.data;

  if (V2 > 0.0001) {
    analogWrite(RPWM2, V2);
    analogWrite(LPWM2, 0);
  }
  else if(V2 < -0.0001){
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, -V2);
  }
  else {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
  }
}

//void motor3_Cb(const std_msgs::Float64 &motor3) {
//  V3 = motor3.data;
//
//  if (V3 > 0.0001) {
//    analogWrite(RPWM3, V3);
//    analogWrite(LPWM3, 0);
//  }
//  else if(V2 < -0.0001){
//    analogWrite(RPWM3, 0);
//    analogWrite(LPWM3, -V3);
//  }
//  else {
//    analogWrite(RPWM2, 0);
//    analogWrite(LPWM2, 0);
//  }
//}

ros::Subscriber<std_msgs::Float64> m1("/sena/right_joint_velocity_controller/command", &motor1_Cb);
ros::Subscriber<std_msgs::Float64> m2("/sena/left_joint_velocity_controller/command", &motor2_Cb);
//ros::Subscriber<std_msgs::Float64> m3("/sena/back_joint_velocity_controller/command", &motor3_Cb);

void setup() {
  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);
  pinMode (RPWM2, OUTPUT);
  pinMode (LPWM2, OUTPUT);
//  pinMode (RPWM3, OUTPUT);
//  pinMode (LPWM3, OUTPUT);

  nh.initNode();
  nh.subscribe(m1);
  nh.subscribe(m2);
//  nh.subscribe(m3);
  //  nh.subscribe(sub_ball);
}

void loop() {

  nh.spinOnce();
  delay(1);
}
