#include <config.h>

float PID1, PID2;
float et, et_1;
float eint, eint_1, eint_update;
float edif;
float Kp, Ti, Td, Ki, Kd;
float SV, PV;
int MV;
unsigned long t;
double t_1, Ts;
float interval_elapsed;
float interval_limit;

float setPoint = 1000;
float incomingSetP = 0;
float maxRpm = 800;

int enc, enc_last, rrpm;
volatile int encoderPos_1;
int enc_dir_now;
int enc_dir_prev;

double error = 0, last_error = 0, sum_error = 0;
float rpm, ppr = 7;
long pm = 0, pm2 = 0;

void setup() {

  Serial.begin(9600);

  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);
  pinMode (ENCO1A, INPUT_PULLUP);
  pinMode (ENCO1B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO1A), readEncoder1, RISING);

  Kp = 0.031;  //0.65
  Ti = 22;   //200
  Td = 0.78;   //0.4

  if (Ti == 0) {
    Ki = 0; //untuk menghindari error akibat pembagian 0
  }
  else {
    Ki = Kp / Ti;
  }
  //--> Hitung Kd
  Kd = Kp * Td;
  et_1 = 0;
  eint_1 = 0;

  interval_limit = 0.05;
  interval_elapsed = 0;

  t = millis();
  delay(10);
}

void loop() {
  read_rpm();

  SV = setPoint;
  PV = rpm;

  t = millis();
  Ts = (t - t_1) / 10;

  et = SV - PV;

  eint_update = ((et + et_1) * Ts) / 2;
  eint = eint_1 + eint_update;
  edif = (et - et_1) / Ts;

  PID1 = Kp * et + Ki * eint + Kd * edif;

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
  MV = PID1 * 255;

  PID2 = PID1;
  analogWrite(RPWM1, MV);

  interval_elapsed = interval_elapsed + Ts;

  if (interval_elapsed >= interval_limit) {
    Serial.print("SV : "); Serial.print(SV); Serial.print("\t");
    Serial.print("PV : "); Serial.println(PV);
  }

  et_1 = et;
  eint_1 = eint;
  t_1 = t;
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

void read_rpm() {
  pm = millis();

  if (pm - pm2 >= 100) {

    rpm = abs((encoderPos_1 / ppr) * 600);

    encoderPos_1 = 0;
    //    Serial.println(rpm);
    pm2 = pm;

  }
}
