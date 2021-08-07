void pid_m3() {
  SV3 = abs(setPointRpm3);
  PV3 = rpm_m3;

  et3 = SV3 - PV3;
  eint3_update = ((et3 + et3_prev) * Ts) / 2;
  eint3 = eint3_prev + eint3_update;
  edif3 = (et3 - et3_prev) / Ts;

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
}
