// kontrol with teleop_keyboard_omni3
// stop = value 0.000001 or 10^-6
// stop = value -0.000001 or -(10^-6)
// non zero stop

#include <ros.h>
#include <std_msgs/Float64.h>
#include <config.h>

// -- PID M1
float PID1;
float et1, et1_prev;
float eint1, eint1_prev, eint1_update;
float edif1;
// -- PID M2
float PID2;
float et2, et2_prev;
float eint2, eint2_prev, eint2_update;
float edif2;
// -- PID M3
float PID3;
float et3, et3_prev;
float eint3, eint3_prev, eint3_update;
float edif3;

float setPointRpm1, setPointRpm2, setPointRpm3;
float setPointVel1, setPointVel2, setPointVel3;

float Kp, Ti, Td, Ki, Kd;
float SV1, SV2, SV3, PV1, PV2, PV3;
int MV1, MV2, MV3;

unsigned long t, t1, t2, t3;
double t_prev, t1_prev, t2_prev, t3_prev, Ts, Ts1, Ts2, Ts3;
unsigned long previousMillis1 = 0, previousMillis2 = 0;

float interval_elapsed;
float interval_limit;
const long interval1, interval2 = 100;

float maxRpm = 800;

volatile int encoderPos_1, encoderPos_2, encoderPos_3;
float rpm_m1, rpm_m2, rpm_m3, pwm1, pwm2, pwm3, ppr = 7;
long pm = 0, pm2 = 0;
// ============== ROS =============
ros::NodeHandle nh;
std_msgs::Float64 rpm1_state;
std_msgs::Float64 rpm2_state;
std_msgs::Float64 rpm3_state;

std_msgs::Float64 vel1_state;
std_msgs::Float64 vel2_state;
std_msgs::Float64 vel3_state;

std_msgs::Float64 pid_1;
std_msgs::Float64 pid_2;
std_msgs::Float64 pid_3;

ros::Publisher rpm1("/rpm_m1", &rpm1_state); // -- rpm
ros::Publisher rpm2("/rpm_m2", &rpm2_state);
ros::Publisher rpm3("/rpm_m3", &rpm3_state);

ros::Publisher vel1("/vel_m1", &vel1_state); // -- vel (pwm)
ros::Publisher vel2("/vel_m2", &vel2_state);
ros::Publisher vel3("/vel_m3", &vel3_state);

ros::Publisher pid1("/pid_m1", &pid_1); // -- pid
ros::Publisher pid2("/pid_m2", &pid_2);
ros::Publisher pid3("/pid_m3", &pid_3);

void setPoint_Cb1(std_msgs::Float64 &msg_motor1) {
  setPointVel1 = msg_motor1.data;
  setPointRpm1 = setPointVel1 * 9.5492965964254;

//  if (setPointVel1 <= 3 and setPointVel1 >= -3) {
//    setPointRpm1 = 0;
//  }
//  else {
//    setPointRpm1 = setPointVel1 * 9.5492965964254; // rad/s to RPM
//  }
}

void setPoint_Cb2(std_msgs::Float64 &msg_motor2) {
  setPointVel2 = msg_motor2.data;
  setPointRpm2 = setPointVel2 * 9.5492965964254;
//  if (setPointVel2 <= 3 and setPointVel2 >= -3) {
//    setPointRpm2 = 0;
//  }
//  else {
//    setPointRpm2 = setPointVel2 * 9.5492965964254; // rad/s to RPM
//  }
}

void setPoint_Cb3(std_msgs::Float64 &msg_motor3) {
  setPointVel3 = msg_motor3.data;

//  if (setPointVel3 <= 3 and setPointVel3 >= -3) {
//    setPointRpm3 = 0;
//  }
//  else {
//    setPointRpm3 = setPointVel3 * 9.5492965964254; // rad/s to RPM
//  }
}

void motor1_Cb(std_msgs::Float64 &m1) {
  pwm1 = m1.data;
  if (pwm1 > 0.0001) {
    pwm1 *= 255;
    analogWrite(RPWM1, pwm1);
    analogWrite(LPWM1, 0);
  }
  else if (pwm1 < -0.0001) {
    pwm1 *= 255;
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -pwm1);
  }
  else {
    pwm1 *= 255;
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
  }
}

void motor2_Cb(std_msgs::Float64 &m2) {
  pwm2 = m2.data;
  if (pwm2 > 0.00001) {
    pwm2 *= 255;
    analogWrite(RPWM2, pwm2);
    analogWrite(LPWM2, 0);
  }
  else if (pwm2 < -0.00001) {
    pwm2 *= 255;
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, -pwm2);
  }
  else {
    pwm2 *= 255;
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
  }
}

ros::Subscriber<std_msgs::Float64> m1("/sena/right_joint_velocity_controller/command", setPoint_Cb1);
ros::Subscriber<std_msgs::Float64> m2("/sena/left_joint_velocity_controller/command", setPoint_Cb2);
ros::Subscriber<std_msgs::Float64> pwm_m1("/pid_m1", motor1_Cb);
ros::Subscriber<std_msgs::Float64> pwm_m2("/pid_m2", motor2_Cb);
// ============== ROS =============

void setup() {
  setup_output();

  nh.initNode();
  
  nh.advertise(rpm1);
  nh.advertise(vel1);
  nh.advertise(pid1);
  nh.subscribe(m1);
  nh.subscribe(pwm_m1);

  nh.advertise(vel2);
  nh.advertise(rpm2);
  nh.advertise(pid2);
  nh.subscribe(m2);
  nh.subscribe(pwm_m2);

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

  et2_prev = 0;
  eint2_prev = 0;

  et3_prev = 0;
  eint3_prev = 0;

  interval_limit = 0.05;
  interval_elapsed = 0;
  t = millis();
//  t1 = millis();
//  t2 = millis();
//  t3 = millis();
}

void loop() {

  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  t = millis();
  Ts = (t - t_prev) / 10;
  
  pid_m1();
  pid_m2();
//  pid_m3();

  t_prev = t;

// 3 ------ MOTOR -------
  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;
    if (setPointVel1 > 0.0000001) {
      vel1_state.data = MV1;
      vel1.publish(&vel1_state);
      pid_1.data = PID1;
      pid1.publish(&pid_1);
    }
    else if (setPointVel1 < -0.0000001){
      vel1_state.data = (MV1 * -1);
      vel1.publish(&vel1_state);
      pid_1.data = (PID1 * -1);
      pid1.publish(&pid_1);
    }
    else {
      pid_1.data = 0;
      pid1.publish(&pid_1);
      vel1_state.data = 0;
      vel1.publish(&vel1_state);
    }
    // -- Motor 2 --
    if (setPointVel2 > 0.0000001) {
      vel2_state.data = MV2;
      vel2.publish(&vel2_state);
      pid_2.data = PID2;
      pid2.publish(&pid_2);
    }
    else if (setPointVel2 < -0.0000001) {
      vel2_state.data = (MV2 * -1);
      vel2.publish(&vel2_state);
      pid_2.data = (PID2 * -1);
      pid2.publish(&pid_2);
    }
    else {
      vel2_state.data = 0;
      vel2.publish(&vel2_state);
      pid_2.data = 0;
      pid2.publish(&pid_2);
    }
  }

  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    rpm_m1 = abs((encoderPos_1 / ppr) * 600);
    rpm1_state.data = setPointRpm1;
    rpm1.publish(&rpm1_state);
    encoderPos_1 = 0;
    
    rpm_m2 = abs((encoderPos_2 / ppr) * 600);
    rpm2_state.data = setPointRpm2;
    rpm2.publish(&rpm2_state);
    encoderPos_2 = 0;
  }
// 3 ------ MOTOR -------

  nh.spinOnce();
  delay(100);
}
