// publish motor value and encoder value
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <config.h>

volatile int64_t encoderPos_1 = 0;

ros::NodeHandle nh;
std_msgs::Int64 encoder1;

ros::Publisher encoder_m1("encoderValue_M1", &encoder1);

void pwm_input(const std_msgs::Int16 &pwm_value) {
  int pwm = 0;
  pwm = pwm_value.data;

  if (pwm > 0) {
    motor1(RPWM1, pwm);
  }
  else {
    motor1(LPWM1, pwm);
  }
}

ros::Subscriber<std_msgs::Int16> pwm("pwm_values", &pwm_input);

void setup() {
  nh.initNode();
  nh.advertise(encoder_m1);
  nh.subscribe(pwm);

  pinMode (ENCO1A, INPUT_PULLUP);
  pinMode (ENCO1B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO1A), readEncoder1, RISING);
  
  TCCR1B = TCCR1B & 0b11111000 | 1;
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

void loop() {
  nh.loginfo("rosserial connection successfully");
  encoder1.data = encoderPos_1;
  encoder_m1.publish(&encoder1);
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
