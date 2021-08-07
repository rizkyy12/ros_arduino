// kontrol with teleop_keyboard_omni3
// ver 2
// stop = value 0.000001 or 10^-6
// stop = value -0.000001 or -(10^-6)
// non zero stop

#include <ros.h>
#include <std_msgs/Float64.h>
#include <config.h>

// -- PID M2
float PID2;
float et2, et2_prev;
float eint2, eint2_prev, eint2_update;
float edif2;

float setPointRpm2, setPointVel2;

float Kp, Ti, Td, Ki, Kd;
float SV1, PV1, SV2, PV2;
int MV1, MV2;

unsigned long t;
double t_prev, Ts;
unsigned long previousMillis1 = 0, previousMillis2 = 0;
const long interval1 = 100;
const long interval2 = 100;

float interval_elapsed;
float interval_limit;

float maxRpm = 800;

volatile int encoderPos_1, encoderPos_2, encoderPos_3;
float rpm_m2, pwm2, ppr = 7;
long pm = 0, pm2 = 0;
// ============== ROS =============
ros::NodeHandle nh;

std_msgs::Float64 vel2_state;
std_msgs::Float64 pid_2;
std_msgs::Float64 rpm2_state;

ros::Publisher vel2("/vel_m2", &vel2_state);
ros::Publisher pid2("/pid_m2", &pid_2);
ros::Publisher rpm2("/rpm_m2", &rpm2_state);

void setPoint_Cb2(std_msgs::Float64 &msg_motor) {
  setPointVel2 = msg_motor.data;
  if (setPointVel2 <= 3.0 and setPointVel2 >= -3.0) {
    setPointRpm2 = 0;
  }
  else {
    setPointRpm2 = setPointVel2 * 9.5492965964254;
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

ros::Subscriber<std_msgs::Float64> m2("/sena/left_joint_velocity_controller/command", setPoint_Cb2);
ros::Subscriber<std_msgs::Float64> pwm_m2("/pid_m2", motor2_Cb);
// ============== ROS =============

void setup() {
  setup_output();

  nh.initNode();

  nh.advertise(vel2);
  nh.advertise(rpm2);
  nh.advertise(pid2);
  nh.subscribe(m2);
  nh.subscribe(pwm_m2);

  Kp = 0.01021;  //0.65
  Ti = 8.057;   //200
  Td = 1.071;   //0.4

  if (Ti == 0) {
    Ki = 0; //untuk menghindari error akibat pembagian 0
  }
  else {
    Ki = Kp / Ti;
  }
  //--> Hitung Kd
  Kd = Kp * Td;

  et2_prev = 0;
  eint2_prev = 0;

  interval_limit = 0.05;
  interval_elapsed = 0;

  t = millis();
}

void loop() {
  //  Serial.begin(9600);
  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();
  
  t = millis();
  Ts = (t - t_prev) / 10;

  //  // var SV1 merupakan nilai absolute/mutlak dari setPointRpm1
  //  // jadi keluaran yang dihasilkan dari PID1 akan selalu +
 
  pid_m2();

  t_prev = t;

  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;
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
    rpm_m2 = abs((encoderPos_2 / ppr) * 600);
    rpm2_state.data = setPointRpm2;
    rpm2.publish(&rpm2_state);
    encoderPos_2 = 0;
  }

  nh.spinOnce();
  delay(100);
}
