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

unsigned long t;
double t_prev, Ts;
unsigned long previousMillis1 = 0, previousMillis2 = 0;

float interval_elapsed;
float interval_limit;
const long interval1, interval2 = 100;

float maxRpm = 800;

volatile int encoderPos_1, encoderPos_2, encoderPos_3;
float rpm_m1, rpm_m2, rpm_m3, ppr = 7;
long pm = 0, pm2 = 0;
// ============== ROS =============
ros::NodeHandle nh;
std_msgs::Float64 rpm1_state;
std_msgs::Float64 rpm2_state;
std_msgs::Float64 rpm3_state;

std_msgs::Float64 vel1_state;
std_msgs::Float64 vel2_state;
std_msgs::Float64 vel3_state;

ros::Publisher rpm1("/rpm_m1", &rpm1_state); // -- rpm
ros::Publisher rpm2("/rpm_m2", &rpm2_state);
ros::Publisher rpm3("/rpm_m3", &rpm3_state);

ros::Publisher vel1("/vel_m1", &vel1_state); // -- vel (pwm)
ros::Publisher vel2("/vel_m2", &vel2_state);
ros::Publisher vel3("/vel_m3", &vel3_state);

void setPoint_Cb1(std_msgs::Float64 &msg_motor1) {
  setPointVel1 = msg_motor1.data;

  if (setPointVel1 <= 3 and setPointVel1 >= -3) {
    setPointRpm1 = 0;
  }
  else {
    setPointRpm1 = setPointVel1 * 9.5492965964254; // rad/s to RPM
    if (setPointRpm1 > 0) {
      motor1(RPWM1, MV1);
      motor1(LPWM1, 0);
    }
    else {
      motor1(RPWM1, 0);
      motor1(LPWM1, MV1);
    }
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

void setPoint_Cb3(std_msgs::Float64 &msg_motor3) {
  setPointVel3 = msg_motor3.data;

  if (setPointVel3 <= 3 and setPointVel3 >= -3) {
    setPointRpm3 = 0;
  }
  else {
    setPointRpm3 = setPointVel3 * 9.5492965964254; // rad/s to RPM
  }
}

ros::Subscriber<std_msgs::Float64> m1("/sena/right_joint_velocity_controller/command", setPoint_Cb1);
ros::Subscriber<std_msgs::Float64> m2("/sena/left_joint_velocity_controller/command", setPoint_Cb2);
ros::Subscriber<std_msgs::Float64> m3("/sena/back_joint_velocity_controller/command", setPoint_Cb3);
// ============== ROS =============

void setup() {
  setup_output();

  nh.initNode();
  nh.advertise(rpm1);
  nh.advertise(vel1);
  nh.subscribe(m1);
  nh.advertise(rpm2);
  nh.advertise(vel2);
  nh.subscribe(m2);
  nh.advertise(rpm3);
  nh.advertise(vel3);
  nh.subscribe(m3);

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
}

void loop() {

  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  SV1 = abs(setPointRpm1);
  SV2 = abs(setPointRpm2);
  SV3 = abs(setPointRpm3);
  PV1 = rpm_m1;
  PV2 = rpm_m2;
  PV3 = rpm_m3;

  t = millis();
  Ts = (t - t_prev) / 10;

  et1 = SV1 - PV1;
  eint1_update = ((et1 + et1_prev) * Ts) / 2;
  eint1 = eint1_prev + eint1_update;
  edif1 = (et1 - et1_prev) / Ts;

  et2 = SV2 - PV2;
  eint2_update = ((et2 + et2_prev) * Ts) / 2;
  eint2 = eint2_prev + eint2_update;
  edif2 = (et2 - et2_prev) / Ts;

  et3 = SV3 - PV3;
  eint3_update = ((et3 + et3_prev) * Ts) / 2;
  eint3 = eint3_prev + eint3_update;
  edif3 = (et3 - et3_prev) / Ts;

  PID1 = Kp * et1 + Ki * eint1 + Kd * edif1;
  PID2 = Kp * et2 + Ki * eint2 + Kd * edif2;
  PID3 = Kp * et3 + Ki * eint3 + Kd * edif3;

  if (PID1, PID2, PID3 > maxRpm) {
    PID1 = maxRpm;
    PID2 = maxRpm;
    PID3 = maxRpm;
  }
  else if (PID1, PID2, PID3 < 0) {
    PID1 = 0;
    PID2 = 0;
    PID3 = 0;
  }
  else {
    PID1 = PID1;
    PID2 = PID2;
    PID3 = PID3;
  }

  PID1 = PID1 / maxRpm;
  MV1 = PID1 * 255;

  PID2 = PID2 / maxRpm;
  MV2 = PID2 * 255;

  PID3 = PID3 / maxRpm;
  MV3 = PID3 * 255;

  analogWrite(RPWM1, MV1);
  analogWrite(RPWM2, MV2);
  analogWrite(RPWM3, MV3);

  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;
    if (setPointRpm1 or setPointRpm2 or setPointRpm3 >= 0) {
      vel1_state.data = MV1;
      vel1.publish(&vel1_state);

      vel2_state.data = MV2;
      vel2.publish(&vel2_state);

      vel3_state.data = MV3;
      vel3.publish(&vel3_state);
    }
    else {
      vel1_state.data = (MV1 * -1);
      vel1.publish(&vel1_state);

      vel2_state.data = (MV2 * -1);
      vel2.publish(&vel2_state);

      vel3_state.data = (MV3 * -1);
      vel3.publish(&vel3_state);
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

    rpm_m3 = abs((encoderPos_3 / ppr) * 600);
    rpm3_state.data = setPointRpm3;
    rpm3.publish(&rpm3_state);
    encoderPos_3 = 0;
  }

  et1_prev = et1;
  eint1_prev = eint1;

  et2_prev = et2;
  eint2_prev = eint2;

  et3_prev = et3;
  eint3_prev = eint3;

  t_prev = t;

  nh.spinOnce();
  delay(100);
}
