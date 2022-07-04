// Motor 1 - K21 ABU Robocon
// PID Speed Control

#define R_PWM1 12
#define L_PWM1 13
#define ENCOA1 20

float Kp_1, Ti_1, Td_1, Ki_1, Kd_1; // Variable Parameters
float interval_elapsed_1;
float interval_limit_1;

unsigned long lasttimepub, now;

float PID1;
float et1, et1_prev;
float eint1, eint1_prev, eint1_update;
float edif1;
float setPointVel1;
float SV1, PV1;
float setPointRpm1 = 3000;

int MV1;
unsigned long t1;
double t_1, Ts1;
float maxRpm = 475;
volatile int encoderPos_1 = 0, encReal_Pos1 = 0;
float V1, rpm_m1, ppr = 7;
unsigned long previousMillis1 = 0, previousMillis2 = 0;
const long interval1, interval2 = 100;

void setup() {
  Serial.begin(115200);

  pinMode(R_PWM1, OUTPUT);
  pinMode(L_PWM1, OUTPUT);
  pinMode(ENCOA1, INPUT_PULLUP);

  attachInterrupt (digitalPinToInterrupt (ENCOA1), readEncoder1, RISING);

  Kp_1 = 0.04821;  //0.01021 //parameters Proportional
  Ti_1 = 7.957;   //8.057     //parameters Integral
  Td_1 = 0.0000027;   //1.071     //parameters Derivative

  // --> Menghitung Ki
  if (Ti_1 == 0) {
    Ki_1 = 0;
  }
  else {
    Ki_1 = Kp_1 / Ti_1;
  }

  // --> Menghitung Kd
  Kd_1 = Kp_1 * Td_1;
  et1_prev = 0;
  eint1_prev = 0;

  interval_limit_1 = 0.05;
  interval_elapsed_1 = 0;

  delay(10);
}

void loop() {
  read_rpm1();

  now = millis();

  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  SV1 = abs(setPointRpm1); // -- Set Point Speed Motor
  PV1 = rpm_m1;            // -- Process Variable

  t1 = millis(); // Start time t1

  Ts1 = (t1 - t_1) / 10;

  et1 = SV1 - PV1;                                // -- Calculate Error
  eint1_update = ((et1 + et1_prev) * Ts1) / 2;    // -- Error Integral
  eint1 = eint1_prev + eint1_update;
  edif1 = (et1 - et1_prev) / Ts1;                 // -- Error Derivative

  PID1 = Kp_1 * et1 + Ki_1 * eint1 + Kd_1 * edif1;     // -- Calculate PID

  if (PID1 > maxRpm) {                           // -- If the PID greather than maxRpm,
    PID1 = maxRpm;                               // -- Set value PID1 to maxRpm
  }
  else if (PID1 < 0) {
    PID1 = 0;
  }

  else {
    PID1 = PID1;
  }

  PID1 = PID1 / maxRpm;
  MV1 = PID1 * 255;                             // -- Measured Value * 255 (PWM)
  analogWrite(R_PWM1, MV1);

  interval_elapsed_1 += Ts1;
  if (interval_elapsed_1 >= interval_limit_1) {
    Serial.print("SV: "); Serial.print(SV1); Serial.print("\t");
    Serial.print("PV: "); Serial.print(PV1); Serial.print("\t");
    Serial.print("MV: "); Serial.println(MV1);
  }

  et1_prev = et1;
  eint1_prev = eint1;
  t_1 = t1;
  delay(100);
}

// Read RPM Motor
void read_rpm1() {
  previousMillis1 = millis();

  if (previousMillis1 - previousMillis2 >= 100) {
    rpm_m1 = abs((encoderPos_1 / ppr) * 600);
    encoderPos_1 = 0;
    previousMillis2 = previousMillis1;
  }
}

// Read Encoder
void readEncoder1() {
  int a = digitalRead(ENCOA1);
  if (a > 0) {
    encoderPos_1--;
  }
  else {
    encoderPos_1++;
  }
}
