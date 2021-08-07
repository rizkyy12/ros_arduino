// control with teleop_keyboard_omni3
// stop = value 0.000001 or 10^-6
// stop = value -0.000001 or -(10^-6)
// non zero stop

#include <ros.h>
#include <std_msgs/Float64.h>
#include <config.h>

float Kp, Ti, Td, Ki, Kd;
float interval_elapsed;
float interval_limit;

// -- PID M1
float PID1;
float et1, et1_prev;
float eint1, eint1_prev, eint1_update;
float edif1;
float setPointVel1, SV1, PV1;
float setPointRpm1;
int MV1;
unsigned long t1;
double t_1, Ts1;
float maxRpm = 800;
volatile int encoderPos_1;
float V1, rpm_m1, ppr = 7;
unsigned long previousMillis1 = 0, previousMillis2 = 0;
const long interval1, interval2 = 100;

// -- PID M2
float PID2;
float et2, et2_prev;
float eint2, eint2_prev, eint2_update;
float edif2;
float setPointVel2, SV2, PV2;
float setPointRpm2;
int MV2;
unsigned long t2;
double t_2, Ts2;
volatile int encoderPos_2;
float V2, rpm_m2;
unsigned long previousMillis3 = 0, previousMillis4 = 0;
const long interval3, interval4 = 100;

// --- PID M3
float PID3;
float et3, et3_prev;
float eint3, eint3_prev, eint3_update;
float edif3;
float setPointVel3, SV3, PV3;
float setPointRpm3;
int MV3;
unsigned long t3;
double t_3, Ts3;
volatile int encoderPos_3;
float V3, rpm_m3;
unsigned long previousMillis5 = 0, previousMillis6 = 0;
const long interval5, interval6 = 100;

// ============== ROS =============
ros::NodeHandle nh;
std_msgs::Float64 rpm1_state;
std_msgs::Float64 vel1_state;
std_msgs::Float64 pid_1;

std_msgs::Float64 rpm2_state;
std_msgs::Float64 vel2_state;
std_msgs::Float64 pid_2;

std_msgs::Float64 rpm3_state;
std_msgs::Float64 vel3_state;
std_msgs::Float64 pid_3;

ros::Publisher rpm1("/rpm_m1", &rpm1_state); // -- rpm
ros::Publisher vel1("/vel_m1", &vel1_state); // -- vel (pwm)
ros::Publisher pid1("/pid_m1", &pid_1); // -- pid

ros::Publisher rpm2("/rpm_m2", &rpm2_state); // -- rpm
ros::Publisher vel2("/vel_m2", &vel2_state); // -- vel (pwm)
ros::Publisher pid2("/pid_m2", &pid_2); // -- pid

ros::Publisher rpm3("/rpm_m3", &rpm3_state); // -- rpm
ros::Publisher vel3("/vel_m3", &vel3_state); // -- vel (pwm)
ros::Publisher pid3("/pid_m3", &pid_3); // -- pid

void setPoint_Cb1(std_msgs::Float64 &msg_motor1) {
  setPointVel1 = msg_motor1.data;
  if (setPointVel1 <= 3 and setPointVel1 >= -3) {
    setPointRpm1 = 0;
  }
  else {
    setPointRpm1 = setPointVel1 * 9.5492965964254; // rad/s to RPM
  }
}

void setPoint_Cb2(std_msgs::Float64 &msg_motor2) {
  setPointVel2 = msg_motor2.data;
  if (setPointVel2 <= 3 and setPointVel2 >= -3) {
    setPointRpm2 = 0;
  }
  else {
    setPointRpm2 = setPointVel2 * 9.5492965964254; // rad/s to RPM
  }
}

void setPoint_Cb3(std_msgs::Float64 &msg_motor) {
  setPointVel3 = msg_motor.data;
  if (setPointVel3 <= 3.0 and setPointVel3 >= -3.0) {
    setPointRpm3 = 0;
  }
  else {
    setPointRpm3 = setPointVel3 * 9.5492965964254;
  }
}

ros::Subscriber<std_msgs::Float64> m1("/sena/right_joint_velocity_controller/command", setPoint_Cb1);
ros::Subscriber<std_msgs::Float64> m2("/sena/left_joint_velocity_controller/command", setPoint_Cb2);
ros::Subscriber<std_msgs::Float64> m3("/sena/back_joint_velocity_controller/command", setPoint_Cb3);
//ros::Subscriber<std_msgs::Float64> pwm_m1("/pid_m1", motor1_Cb);
//ros::Subscriber<std_msgs::Float64> pwm_m2("/pid_m2", motor2_Cb);
//ros::Subscriber<std_msgs::Float64> pwm_m3("/pid_m3", motor3_Cb);
// ============== ROS =============

void setup() {
  setup_output();
  
  nh.initNode();
  nh.advertise(rpm1);
  nh.advertise(vel1);
  nh.advertise(pid1);
  nh.subscribe(m1);
//  nh.subscribe(pwm_m1);

  Kp = 0.009821;  //0.01021
  Ti = 8.057;   //8.057
  Td = 1.071;   //1.071

  if (Ti == 0) {
    Ki = 0;
  }
  else {
    Ki = Kp / Ti;
  }
  //--> Hitung Kd
  Kd = Kp * Td;
  et1_prev = 0;
  eint1_prev = 0;

  interval_limit = 0.05;
  interval_elapsed = 0;
  t1 = millis();
  delay(10);

  nh.advertise(rpm2);
  nh.advertise(vel2);
  nh.advertise(pid2);
  nh.subscribe(m2);
//  nh.subscribe(pwm_m2);
  t2 = millis();
  delay(10);

  nh.advertise(rpm3);
  nh.advertise(vel3);
  nh.advertise(pid3);
  nh.subscribe(m3);
//  nh.subscribe(pwm_m3);
  t3 = millis();
  delay(10);
}

void loop() {
  // ---- MOTOR 1 -----
  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  pid_m1();
  
  if (currentMillis1 - previousMillis1 >= interval1) {
    // save the last time you blinked the LED
    previousMillis1 = currentMillis1;

    if (setPointVel1 > 0.00001 ) {
      pid_1.data = PID1;
      pid1.publish (&pid_1);
      analogWrite(RPWM1, MV1);
      analogWrite(LPWM1, 0);
    }
    else if (setPointVel1 < -0.00001){
      pid_1.data = (PID1 * -1);
      pid1.publish(&pid_1);
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, MV1);
    }
    else {
      pid_1.data = 0;
      pid1.publish(&pid_1);
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, 0);
    }
  }
  
  nh.spinOnce();
  
  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    rpm_m1 = abs((encoderPos_1 / ppr) * 600);
    rpm1_state.data = setPointRpm1;
    rpm1.publish(&rpm1_state);
    encoderPos_1 = 0;
  }
  
  nh.spinOnce();

  // ---- MOTOR 2 -----

  unsigned long currentMillis3 = millis();
  unsigned long currentMillis4 = millis();

  pid_m2();
  
  if (currentMillis3 - previousMillis3 >= interval3) {
    
    previousMillis3 = currentMillis3;

    if (setPointVel2 > 0.00001 ) {
      pid_2.data = PID2;
      pid2.publish (&pid_2);
      analogWrite(RPWM2, MV2);
      analogWrite(LPWM2, 0);
    }
    else if (setPointVel2 < -0.00001){
      pid_2.data = PID2;
      pid2.publish(&pid_2);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, MV2);
    }
    else{
      pid_2.data = 0;
      pid2.publish(&pid_2);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, MV2);
    }
  }
  
  nh.spinOnce();
  
  if (currentMillis4 - previousMillis4 >= interval4) {
    previousMillis4 = currentMillis4;
    rpm_m2 = abs((encoderPos_2 / ppr) * 600);
    rpm2_state.data = setPointRpm2;
    rpm2.publish(&rpm2_state);
    encoderPos_2 = 0;
  }

  // ---- MOTOR 3 -----

  unsigned long currentMillis5 = millis();
  unsigned long currentMillis6 = millis();

  pid_m3();
  
  if (currentMillis5 - previousMillis5 >= interval5) {
    
    previousMillis5 = currentMillis5;

    if (setPointVel3 > 0.00001) {
      pid_3.data = MV3;
      pid3.publish (&pid_3);
      analogWrite(RPWM3, MV3);
      analogWrite(LPWM3, 0);
    }
    else if(setPointVel3 < -0.00001){
      pid_3.data = (MV3 * -1);
      pid3.publish(&pid_3);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, MV3);
    }
    else {
      pid_3.data = 0;
      pid3.publish(&pid_3);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, 0);
    }
  }
  
  nh.spinOnce();
  
  if (currentMillis6 - previousMillis6 >= interval6) {
    previousMillis6 = currentMillis6;
    rpm_m3 = abs((encoderPos_3 / ppr) * 600);
    rpm3_state.data = setPointRpm3;
    rpm3.publish(&rpm3_state);
    encoderPos_3 = 0;
  }
  nh.spinOnce();
  delay(100);  
}
