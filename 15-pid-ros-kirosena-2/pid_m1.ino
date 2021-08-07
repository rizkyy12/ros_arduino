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
  PID2 = PID1;

  et1_prev = et1;
  eint1_prev = eint1;
}

void pid_m2() {
  SV2 = abs(setPointRpm2);
  PV2 = rpm_m2;

  et2 = SV2 - PV2;
  eint2_update = ((et2 + et2_prev) * Ts) / 2;
  eint2 = eint2_prev + eint2_update;
  edif2 = (et2 - et2_prev) / Ts;


  PID2 = Kp * et2 + Ki * eint2 + Kd * edif2;

  if (PID2 > maxRpm) {
    PID2 = maxRpm;
  }

  else if (PID2 < 0) {
    PID2 = 0;
  }

  else {
    PID2 = PID2;
  }

  PID2 = PID2 / maxRpm;
  MV2 = PID2 * 255;

  et2_prev = et2;
  eint2_prev = eint2;
}
