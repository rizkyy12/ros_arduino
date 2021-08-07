#include <ros.h>
#include <rospy_tutorials/Floats.h>

#define ENCA 19
#define ENCB 18

ros::NodeHandle nh;
rospy_tutorials::Floats data_enc;
ros::Publisher pub("/data_enc", &data_enc);

int encoderPos = 0;
unsigned long lasttimepub, now;

void setup() {
  nh.initNode();
  nh.advertise(pub);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  TCCR1B = TCCR1B & 0b11111000 | 1;

}

void loop() {
  now = millis();
  if ((now - lasttimepub) > 100) {
    data_enc.data_length = 1;
    data_enc.data[0] = encoderPos;
    pub.publish(&data_enc);
    lasttimepub = now;
  }
  nh.spinOnce();
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    encoderPos++;
  }
  else {
    encoderPos--;
  }
}
