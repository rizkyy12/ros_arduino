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
