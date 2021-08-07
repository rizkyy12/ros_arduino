// pertama kali
// ketika dari -100 ke 0, RPWM akan ke triger

#include <ros.h>
#include <std_msgs/Float64.h>
#include <config.h>

// -- PID M1
float PID1, PID2;
float et1, et1_prev;
float eint1, eint1_prev, eint1_update;
float edif1;

float setPointRpm1;
float setPointVel1;

float Kp, Ti, Td, Ki, Kd;
float SV1, PV1;
int MV1;

unsigned long t;
double t_prev, Ts;
unsigned long previousMillis1 = 0, previousMillis2 = 0;
const long interval1 = 100;
const long interval2 = 100;

float interval_elapsed;
float interval_limit;

float maxRpm = 800;

volatile int encoderPos_1;
float rpm_m1, pwm1, pid1_m1, ppr = 7;
long pm = 0, pm2 = 0;
// ============== ROS =============
ros::NodeHandle nh;
std_msgs::Float64 vel1_state;
std_msgs::Float64 pid_1;
std_msgs::Float64 rpm1_state;

ros::Publisher vel1("/vel_m1", &vel1_state);
ros::Publisher pid1("/pid_m1", &pid_1);
ros::Publisher rpm1("/rpm_m1", &rpm1_state);

void setPoint_Cb1(std_msgs::Float64 &msg_motor) {
  setPointVel1 = msg_motor.data;
  if (setPointVel1 <= 3.0 and setPointVel1 >= -3.0) {
    setPointRpm1 = 0;
  }
  else {
    setPointRpm1 = setPointVel1 * 9.5492965964254;
  }
}

void motor1_Cb(std_msgs::Float64 &m1) {
  pwm1 = m1.data;
  if (pwm1 >= 0) {
    pwm1 *= 255;
    analogWrite(RPWM1, pwm1);
    analogWrite(LPWM1, 0);
  }
  else {
    pwm1 *= 255;
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, -pwm1);
  }
}

ros::Subscriber<std_msgs::Float64> m1("/sena/right_joint_velocity_controller/command", setPoint_Cb1);
//ros::Subscriber<std_msgs::Float64> pwm_m1("/pid_m1", motor1_Cb);
// ============== ROS =============

void setup() {
  setup_output();

  nh.initNode();
  nh.advertise(vel1);
  nh.advertise(rpm1);
  nh.advertise(pid1);
  nh.subscribe(m1);
//  nh.subscribe(pwm_m1);

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

  et1_prev = 0;
  eint1_prev = 0;

  interval_limit = 0.05;
  interval_elapsed = 0;

  t = millis();
}

void loop() {
  //  Serial.begin(9600);
  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  read_rpm();

  SV1 = abs(setPointRpm1);
  PV1 = rpm_m1;

  t = millis();
  Ts = (t - t_prev) / 10;

  et1 = SV1 - PV1;
  eint1_update = ((et1 + et1_prev) * Ts) / 2;
  eint1 = eint1_prev + eint1_update;
  edif1 = (et1 - et1_prev) / Ts;

  PID1 = Kp * et1 + Ki * eint1 + Kd * edif1;

  if (PID1 > maxRpm) {
    PID1 = maxRpm;
  }

  else if (PID1 < 0) {
    PID1 = 0;
  }

  else {
    PID1 = PID1;
  }

  PID1 = PID1 / maxRpm;
  MV1 = PID1 * 255;

  // var SV1 merupakan nilai absolute/mutlak dari setPointRpm1
  // jadi keluaran yang dihasilkan dari PID1 akan selalu +

  et1_prev = et1;
  eint1_prev = eint1;

  t_prev = t;

  if (currentMillis1 - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis1;
    if (setPointVel1 > 0) {
      vel1_state.data = MV1;
      vel1.publish(&vel1_state);
      pid_1.data = PID1;
      pid1.publish(&pid_1);
      analogWrite(RPWM1, MV1);
      analogWrite(LPWM1, 0);
    }
    else {
      vel1_state.data = (MV1 * -1);
      vel1.publish(&vel1_state);
      pid_1.data = (PID1 * -1);
      pid1.publish(&pid_1);
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, MV1);
    }
  }
  nh.spinOnce();
  delay(100);
}

void read_rpm() {
  pm = millis();

  if (pm - pm2 >= 100) {

    rpm_m1 = abs((encoderPos_1 / ppr) * 600);
    rpm1_state.data = setPointRpm1;
    rpm1.publish(&rpm1_state);
    encoderPos_1 = 0;
    //    Serial.println(rpm);
    pm2 = pm;

  }
}
