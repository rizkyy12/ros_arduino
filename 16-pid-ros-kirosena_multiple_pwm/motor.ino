void motor1(int64_t direct, uint64_t MV1) {
  if (direct == RPWM1) {
    analogWrite(RPWM1, MV1);
    analogWrite(LPWM1, 0);
  }
  else if (direct == LPWM1) {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, MV1);
  }
}

void motor2(int64_t direct, uint64_t MV2) {
  if (direct == RPWM2) {
    analogWrite(RPWM2, MV2);
    analogWrite(LPWM2, 0);
  }
  else if (direct == LPWM2) {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, MV2);
  }
}

void motor1(int64_t direct, uint64_t MV3) {
  if (direct == RPWM3) {
    analogWrite(RPWM3, MV3);
    analogWrite(LPWM3, 0);
  }
  else if (direct == LPWM3) {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, MV3);
  }
}
