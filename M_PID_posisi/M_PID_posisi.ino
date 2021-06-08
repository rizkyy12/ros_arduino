#define ENCA 2 //red
#define ENCB 3 //black
#define RPWM 5
#define LPWM 4

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;


void setup() {

  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEnc, RISING);
  Serial.println("target pos ");
}

void loop() {
  // set target position
  int target = 300;
  //PID Constants
  float kp = 2.6;
  float kd = 0.17;
  float ki = 0.02;

  //time difference
  long currT = micros();
  long et = currT - prevT;
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  //error
  int e = pos - target;

  // derivative
  float dt = (e - eprev) / (deltaT);
  // integral
  eintegral = eintegral + e * deltaT;

  // kontrol signal
  float u = kp * e + kd * dt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  //store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" << Target, Posisi >> ");
  Serial.print(pos);
  Serial.println();
  //signal the motor
  setMotor(dir, pwr, LPWM, RPWM);
}

void setMotor(int dir, int pwmVal, int lpwmVal, int rpwmVal) {
  if (dir == 1) {
    analogWrite(RPWM, pwmVal);
    analogWrite(LPWM, 0);
  }
  else if (dir == -1) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwmVal);
  }
  else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void readEnc() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  }
  else {
    pos--;
  }
}
