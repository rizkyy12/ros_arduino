#include <ros.h>
#include <geometry_msgs/Twist.h>

#define R_PWM1 5
#define L_PWM1 6
#define R_PWM2 7
#define L_PWM2 8
#define R_PWM3 9
#define L_PWM3 10
#define R_PWM4 11
#define L_PWM4 12

int Vx, Vy, w;
ros::NodeHandle nh;

void motorCb(const geometry_msgs::Twist &msg) {
  Vx = msg.linear.x;
  Vy = msg.linear.y;
  w = msg.linear.z;
//
  if (Vy >= 0 && Vx == 0) {
    maju(Vy);
  }
  if(Vy <= 0 && Vx == 0) {
    mundur(Vy);
  }

  if (Vx >= 0 && Vy == 0){
    kanan(Vx);
  }
  if(Vx <= 0 && Vy == 0){
    kiri(Vx);
  }
}

void maju(uint16_t pwm) {
  analogWrite(R_PWM1, pwm);
  analogWrite(L_PWM1, 0);
  analogWrite(R_PWM2, 0);
  analogWrite(L_PWM2, pwm);
  analogWrite(R_PWM3, 0);
  analogWrite(L_PWM3, pwm);
  analogWrite(R_PWM4, pwm);
  analogWrite(L_PWM4, 0);
  
}
//
void mundur(uint16_t pwm) {
  analogWrite(R_PWM1, 0);
  analogWrite(L_PWM1, -pwm);
  analogWrite(R_PWM2, -pwm);
  analogWrite(L_PWM2, 0);
  analogWrite(R_PWM3, -pwm);
  analogWrite(L_PWM3, 0);
  analogWrite(R_PWM4, 0);
  analogWrite(L_PWM4, -pwm);
}

void kanan(uint16_t pwm) {
  analogWrite(R_PWM1, pwm);
  analogWrite(L_PWM1, 0);
  analogWrite(R_PWM2, pwm);
  analogWrite(L_PWM2, 0);
  analogWrite(R_PWM3, 0);
  analogWrite(L_PWM3, pwm);
  analogWrite(R_PWM4, 0);
  analogWrite(L_PWM4, pwm);
}
void kiri(uint16_t pwm) {
  analogWrite(R_PWM1, 0);
  analogWrite(L_PWM1, -pwm);
  analogWrite(R_PWM2, 0);
  analogWrite(L_PWM2, -pwm);
  analogWrite(R_PWM3, -pwm);
  analogWrite(L_PWM3, 0);
  analogWrite(R_PWM4, -pwm);
  analogWrite(L_PWM4, 0);
}

void rot_kanan(uint16_t pwm) {
  analogWrite(R_PWM1, pwm);
  analogWrite(L_PWM1, 0);
  analogWrite(R_PWM2, pwm);
  analogWrite(L_PWM2, 0);
  analogWrite(R_PWM3, pwm);
  analogWrite(L_PWM3, 0);
  analogWrite(R_PWM4, pwm);
  analogWrite(L_PWM4, 0);
}

void rot_kiri(uint16_t pwm) {
  analogWrite(R_PWM1, 0);
  analogWrite(L_PWM1, pwm);
  analogWrite(R_PWM2, 0);
  analogWrite(L_PWM2, pwm);
  analogWrite(R_PWM3, 0);
  analogWrite(L_PWM3, pwm);
  analogWrite(R_PWM4, 0);
  analogWrite(L_PWM4, pwm);
}


ros::Subscriber<geometry_msgs::Twist> velocity("/cmd_vel", &motorCb);


void setup() {
  // put your setup code here, to run once:

  nh.initNode();
  nh.subscribe(velocity);

  pinMode (R_PWM1, OUTPUT);
  pinMode (L_PWM1, OUTPUT);
  pinMode (R_PWM2, OUTPUT);
  pinMode (L_PWM2, OUTPUT);
  pinMode (R_PWM3, OUTPUT);
  pinMode (L_PWM3, OUTPUT);
  pinMode (R_PWM4, OUTPUT);
  pinMode (L_PWM4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.loginfo("rosserial connection successfully");
  nh.spinOnce();
}
