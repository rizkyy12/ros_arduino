// Gripper new Arduino Mega
// Motor 1 - K21 ABU Robocon
#define R_PWM1 6
#define L_PWM1 7
#define ENCOA1 21
#define ENCOB1 20

int encoderPos_1 = 0;

long curT, prevT, eT = 0;
float ePrev, deltaT = 0;
float dt, eIntegral = 0;
int e;
float Kp, Ki, Kd, PID1;

float interval_elapsed_1;
float interval_limit_1;
unsigned long t1;
double t_1, Ts1;
int dir;

float MV1;

void setup() {
  Serial.begin(115200);
  
  pinMode(R_PWM1, OUTPUT);
  pinMode(L_PWM1, OUTPUT);
  pinMode(ENCOA1, INPUT_PULLUP);
  pinMode(ENCOB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCOA1), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCOB1), readEncoderB, CHANGE);
  
  Serial.println("target pos: ");
  interval_limit_1 = 0.05;
  interval_elapsed_1 = 0;
  delay(10);
}

void loop() {
  
  t1 = millis();
  Ts1 = (t1 - t_1) / 10;
  
  int setPoint1 = 2000;
  
  Kp = 0.3962;
  Ki = 0.005127;
  Kd = 0.021;

  curT = micros();
  eT = curT - prevT;
  deltaT = ((float) (curT - prevT)/1.0e6);
  prevT = curT;

  e = encoderPos_1 - setPoint1;

  dt = (e - ePrev) / deltaT;

  eIntegral = eIntegral + e * deltaT;

  PID1 = Kp * e + Ki * eIntegral + Kd * dt;

  MV1 = fabs(PID1);

  if (MV1 > 255){
    MV1 = 255;
  }

  if(PID1 < 0){
    dir = -1;
  }
  else{
    dir = 1;
  }

  ePrev = e;

  setMotor(dir, MV1, R_PWM1, L_PWM1);
  
  interval_elapsed_1 += Ts1;
  if (interval_elapsed_1 >= interval_limit_1) {
    Serial.print("SV: "); Serial.print(setPoint1); Serial.print("\t");
    Serial.print("PV: "); Serial.println(encoderPos_1); 
  }
  
  t_1 = t1;
  
  delay(100);
}

void setMotor(int dir, int pwmVal, int lpwmVal, int rpwmVal){
  if (dir == 1) {
    analogWrite(R_PWM1, pwmVal);
    analogWrite(L_PWM1, 0);
  }
  else if(dir == -1){
    analogWrite(R_PWM1, 0);
    analogWrite(L_PWM1, pwmVal);
  }
  else{
    analogWrite(R_PWM1, 0);
    analogWrite(L_PWM1, 0);
  }
}

void readEncoderA() {
  int a = digitalRead(ENCOA1);
  int b = digitalRead(ENCOB1);
  if (a == HIGH) {
    if (b == LOW) {
      encoderPos_1++;
    }
    else {
      encoderPos_1--;
    }
  }
  else {
    if (b == HIGH) {
      encoderPos_1++;
    }
    else {
      encoderPos_1--;
    }
  }
}

void readEncoderB() {
  int a = digitalRead(ENCOA1);
  int b = digitalRead(ENCOB1);
  if (b == HIGH) {
    if (a == HIGH) {
      encoderPos_1++;
    }
    else {
      encoderPos_1--;
    }
  }
  else {
    if (a == LOW) {
      encoderPos_1++;
    }
    else {
      encoderPos_1--;
    }
  }
}
