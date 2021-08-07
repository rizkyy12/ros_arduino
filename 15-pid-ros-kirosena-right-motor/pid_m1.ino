void pid_m1() {
  SV1 = abs(setPointRpm1);
  PV1 = rpm_m1;

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

  et1_prev = et1;
  eint1_prev = eint1;
}
