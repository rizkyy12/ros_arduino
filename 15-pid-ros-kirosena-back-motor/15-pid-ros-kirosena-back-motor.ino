// kontrol with teleop_keyboard_omni3
// ver 2
// stop = value 0.000001 or 10^-6
// stop = value -0.000001 or -(10^-6)
// non zero stop

#include <ros.h>
#include <std_msgs/Float64.h>
#include <config.h>

// -- PID M3
float PID3;
float et3, et3_prev;
float eint3, eint3_prev, eint3_update;
float edif3;

float setPointRpm3, setPointVel3;

float Kp, Ti, Td, Ki, Kd;
float SV3, PV3;
int MV3;

unsigned long t;
double t_prev, Ts;
unsigned long previousMillis1 = 0, previousMillis2 = 0;
const long interval1 = 100;
const long interval2 = 100;

float interval_elapsed;
float interval_limit;

float maxRpm = 800;

volatile int encoderPos_1, encoderPos_2, encoderPos_3;
float rpm_m3, pwm3, ppr = 7;
long pm = 0, pm2 = 0;
// ============== ROS =============
ros::NodeHandle nh;

std_msgs::Float64 vel3_state;
std_msgs::Float64 pid_3;
std_msgs::Float64 rpm3_state;

ros::Publisher vel3("/vel_m3", &vel3_state);
ros::Publisher pid3("/pid_m3", &pid_3);
ros::Publisher rpm3("/rpm_m3", &rpm3_state);

void setPoint_Cb3(std_msgs::Float64 &msg_motor) {
  setPointVel3 = msg_motor.data;
  if (setPointVel3 <= 3.0 and setPointVel3 >= -3.0) {
    setPointRpm3 = 0;
  }
  else {
    setPointRpm3 = setPointVel3 * 9.5492965964254;
  }
}

void motor3_Cb(std_msgs::Float64 &m3) {
  pwm3 = m3.data;
  if (pwm3 > 0.0001) {
    pwm3 *= 255;
    analogWrite(RPWM3, pwm3);
    analogWrite(LPWM3, 0);
  }
  else if (pwm3 < -0.00001) {
    pwm3 *= 255;
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, -pwm3);
  }
  else {
    pwm3 *= 255;
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
  }
}

ros::Subscriber<std_msgs::Float64> m3("/sena/back_joint_velocity_controller/command", setPoint_Cb3);
ros::Subscriber<std_msgs::Float64> pwm_m3("/pid_m3", motor3_Cb);
// ============== ROS =============

void setup() {
  setup_output();

  nh.initNode();

  nh.advertise(vel3);
  nh.advertise(rpm3);
  nh.advertise(pid3);
  nh.subscribe(m3);
  nh.subscribe(pwm_m3);

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

  et3_prev = 0;
  eint3_prev = 0;

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
 
  pid_m3();

  t_prev = t;

  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;
    if (setPointVel3 > 0.0000001) {
      vel3_state.data = MV3;
      vel3.publish(&vel3_state);
      pid_3.data = PID3;
      pid3.publish(&pid_3);
    }
    else if (setPointVel3 < -0.0000001) {
      vel3_state.data = (MV3 * -1);
      vel3.publish(&vel3_state);
      pid_3.data = (PID3 * -1);
      pid3.publish(&pid_3);
    }
    else {
      vel3_state.data = 0;
      vel3.publish(&vel3_state);
      pid_3.data = 0;
      pid3.publish(&pid_3);
    }
  }

  if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    rpm_m3 = abs((encoderPos_3 / ppr) * 600);
    rpm3_state.data = setPointRpm3;
    rpm3.publish(&rpm3_state);
    encoderPos_3 = 0;
  }

  nh.spinOnce();
  delay(100);
}
