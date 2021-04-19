#include <ros.h>
#include <rospy_tutorials/Floats.h>

#define ENCA 3
#define ENCB 2
#define LED 13

ros::NodeHandle nh;

double pos = 0, output = 0, temp = 0;
unsigned long lastTime, now, lasttimepub;
volatile long encoderPos = 0, last_pos = 0;

rospy_tutorials::Floats joint_state;

void dataCb(const rospy_tutorials::Floats& msg) {
  output = msg.data[0];
}

ros::Publisher pub("/pub_from_arduino", &joint_state);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
  
  TCCR1B = TCCR1B & 0b11111000 | 1;

}

void loop() {
  pos = (encoderPos * 360) / 448;
  now = millis();
  int timeChange = (now - lastTime);

  if (timeChange >= 500) {
    temp = (360.0 * 1000 * (encoderPos - last_pos)) / (448.0*(now - lastTime));
    
    if ((encoderPos < -1 || encoderPos > 1) && temp >= -60 && temp <= 60) {
      
      lastTime = now;
      last_pos = encoderPos;
    }
  }
  if ((now - lasttimepub) > 100) {
    joint_state.data_length = 1;
    joint_state.data[0] = pos;
    pub.publish(&joint_state);
    lasttimepub = now;
  }
  nh.spinOnce();
}

void readEncoder() {
  int b = digitalRead(ENCA);
  if (b > 0) {
    encoderPos++;
    digitalWrite(LED, HIGH);
  }
  else {
    encoderPos--;
    digitalWrite(LED, LOW);
  }
}
