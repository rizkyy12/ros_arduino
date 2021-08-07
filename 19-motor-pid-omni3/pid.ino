void pid_m1() {
  SV1 = abs(setPointRpm1);
  PV1 = rpm_m1;

  t1 = millis();
  Ts1 = (t1 - t_1) / 10;

  et1 = SV1 - PV1;
  eint1_update = ((et1 + et1_prev) * Ts1) / 2;
  eint1 = eint1_prev + eint1_update;
  edif1 = (et1 - et1_prev) / Ts1;

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
  t_1 = t1;
}

void pid_m2() {
  SV2 = abs(setPointRpm2);
  PV2 = rpm_m2;

  t2 = millis();
  Ts2 = (t2 - t_2) / 10;

  et2 = SV2 - PV2;
  eint2_update = ((et2 + et2_prev) * Ts2) / 2;
  eint2 = eint2_prev + eint2_update;
  edif2 = (et2 - et2_prev) / Ts2;

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
  t_2 = t2;
}

void pid_m3() {
  SV3 = abs(setPointRpm3);
  PV3 = rpm_m3;

  t3 = millis();
  Ts3 = (t3 - t_3) / 10;
  
  et3 = SV3 - PV3;
  eint3_update = ((et3 + et3_prev) * Ts3) / 2;
  eint3 = eint3_prev + eint3_update;
  edif3 = (et3 - et3_prev) / Ts3;

  PID3 = Kp * et3 + Ki * eint3 + Kd * edif3;

  if (PID3 > maxRpm) {
    PID3 = maxRpm;
  }

  else if (PID3 < 0) {
    PID3 = 0;
  }

  else {
    PID3 = PID3;
  }

  PID3 = PID3 / maxRpm;
  MV3 = PID3 * 255;

  et3_prev = et3;
  eint3_prev = eint3;
  t_3 = t3;
}
