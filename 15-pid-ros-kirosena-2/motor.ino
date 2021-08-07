//void motor1(int64_t direct, uint64_t pwm_motor) {
//    if (direct == RPWM1) {
//      analogWrite(RPWM1, MV1);
//      analogWrite(LPWM1, 0);
//    }
//    else if (direct == LPWM1) {s
//      analogWrite(RPWM1, 0);
//      analogWrite(LPWM1, MV1);
//    }
//    else {
//      analogWrite(RPWM1, 0);
//      analogWrite(LPWM1, 0);
//    }
//}

//void motor1() {
//  if (setPointRpm1 >= 0) {
//    if (PID1 > 0) {
//      analogWrite(RPWM1, MV1);
//      analogWrite(LPWM1, 0);
//    }
//  }
//  else {
//    PID1 = PID1 * (-1);
//    if (PID1 < 0) {
//      analogWrite(RPWM1, 0);
//      analogWrite(LPWM1, 0);
//    }
//  }
//}
