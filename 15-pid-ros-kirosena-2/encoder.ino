void readEncoder1() {
  int b = digitalRead(ENCO1B);
  if (b > 0) {
    encoderPos_1++;
  }
  else {
    encoderPos_1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCO2B);
  if (b > 0) {
    encoderPos_2++;
  }
  else {
    encoderPos_2--;
  }
}

void readEncoder3() {
  int b = digitalRead(ENCO3B);
  if (b > 0) {
    encoderPos_3++;
  }
  else {
    encoderPos_3--;
  }
}
