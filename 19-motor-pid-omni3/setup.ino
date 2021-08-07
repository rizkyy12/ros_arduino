void setup_output() {
  pinMode (RPWM1, OUTPUT);
  pinMode (LPWM1, OUTPUT);
  pinMode (ENCO1A, INPUT_PULLUP);
  pinMode (ENCO1B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO1A), readEncoder1, RISING);

  pinMode (RPWM2, OUTPUT);
  pinMode (LPWM2, OUTPUT);
  pinMode (ENCO2A, INPUT_PULLUP);
  pinMode (ENCO2B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO2A), readEncoder2, RISING);

  pinMode (RPWM3, OUTPUT);
  pinMode (LPWM3, OUTPUT);
  pinMode (ENCO3A, INPUT_PULLUP);
  pinMode (ENCO3B, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (ENCO3A), readEncoder3, RISING);
}
