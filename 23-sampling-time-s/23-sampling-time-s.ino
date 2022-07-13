unsigned long time_1 = 0;
unsigned long int INTERVAL_MESSAGE1 = 5000;

// prototype
void print_time(unsigned long time_millis);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  if (millis() >= time_1 + INTERVAL_MESSAGE1){
    time_1 += INTERVAL_MESSAGE1;
    print_time(time_1);
    Serial.println("data ke detik 1");
  }
}

void print_time(unsigned long time_millis){
  Serial.print("Time: ");Serial.print(time_millis/1000);
  Serial.print("s - ");  
}
