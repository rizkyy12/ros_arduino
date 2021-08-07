#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <ArduinoHardware.h>
#include <config.h>

volatile int64_t encoderPos_1 = 0, encoderPos_2 = 0, encoderPos_3 = 0;

ros::NodeHandle nh;
std_msgs::Int64 encoder1;
std_msgs::Int64 encoder2;
std_msgs::Int64 encoder3;

ros::Publisher encoder_m1("/encoderValue_m1", &encoder1);
//ros::Publisher encoder_m2("/encoderValue_m2", &encoder2);
//ros::Publisher encoder_m3("/encoderValue_m3", &encoder3);

void pwm_Cb(const std_msgs::Int64 &msg) {
  int pwm = 0;
  pwm = msg.data;

  if (pwm > 0){
    motor1(RPWM1, pwm);
//    motor2(RPWM2, pwm);
//    motor3(RPWM3, pwm);
  }
  else {
    motor1(LPWM1, pwm);
//    motor2(LPWM2, pwm);
//    motor3(LPWM3, pwm);
  }
}

ros::Subscriber<std_msgs::Int64> pwm_motor("/pwm", &pwm_Cb);

void setup() {
  nh.initNode();
  nh.advertise(encoder_m1);
//  nh.advertise(encoder_m2);
//  nh.advertise(encoder_m3);
  nh.subscribe(pwm_motor);

  pinMode (ENCO1A, INPUT_PULLUP);
  pinMode (ENCO1B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO1A), readEncoder1, RISING);

//  pinMode (ENCO2A, INPUT_PULLUP);
//  pinMode (ENCO2B, INPUT_PULLUP);
//  attachInterrupt (digitalPinToInterrupt (ENCO2A), readEncoder2, RISING);
//
//  pinMode (ENCO3A, INPUT_PULLUP);
//  pinMode (ENCO3B, INPUT_PULLUP);
//  attachInterrupt (digitalPinToInterrupt (ENCO3A), readEncoder3, RISING);
  
}

void motor1(int16_t direct, uint16_t pwm) {
  if (direct == RPWM1) {
    analogWrite(RPWM1, pwm);
    analogWrite(LPWM1, 0);
  }
  else if (direct == LPWM1) {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -pwm);
  }
  else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
  }
}

void motor2(int16_t direct, uint16_t pwm) {
  if (direct == RPWM2) {
    analogWrite(RPWM2, pwm);
    analogWrite(LPWM2, 0);
  }
  else if (direct == LPWM2) {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, -pwm);
  }
  else {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
  }
}

void motor3(int16_t direct, uint16_t pwm) {
  if (direct == RPWM3) {
    analogWrite(RPWM3, pwm);
    analogWrite(LPWM3, 0);
  }
  else if (direct == LPWM3) {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, -pwm);
  }
  else {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
  }
}

void loop() {
  encoder1.data = encoderPos_1;
//  encoder2.data = encoderPos_2;
//  encoder3.data = encoderPos_3;
  encoder_m1.publish(&encoder1);
//  encoder_m2.publish(&encoder2);
//  encoder_m3.publish(&encoder3);

  nh.loginfo("rosserial connection successfully");  
  nh.spinOnce();
  delay(100);
}

void readEncoder1() {
  int b = digitalRead(ENCO1B);
  if (b > 0) {
    encoderPos_1++;
  }
  else {
    encoderPos_1--;
  }
}

//void readEncoder2() {
//  int b = digitalRead(ENCO2B);
//  if (b > 0) {
//    encoderPos_2++;
//  }
//  else {
//    encoderPos_2--;
//  }
//}
//
//void readEncoder3() {
//  int b = digitalRead(ENCO3B);
//  if (b > 0) {
//    encoderPos_3++;
//  }
//  else {
//    encoderPos_3--;
//  }
//}
