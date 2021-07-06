#include <ros.h>
#include <rospy_tutorials/Floats.h>

#define encoA 2
#define encoB 3
#define RPWM 5
#define LPWM 4

double pos = 0, vel = 0, output = 0, temp = 0;
unsigned long lastTime, now, lasttimepub;
volatile long encoderPos = 0, lastPos = 0;

ros::NodeHandle nh;
rospy_tutorials::Floats joint_state;

void set_angle_cb(const rospy_tutorials::Floats &cmd_msg) {
  output = cmd_msg.data[0];
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_arduino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &joint_state);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(encoA, INPUT_PULLUP);
  pinMode(encoB, INPUT_PULLUP);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  attachInterrupt(0, encoder, FALLING);

  TCCR1B = TCCR1B & 0b11111000 | 1;

}

void encoder() {
  if (encoderPos > 2250 || encoderPos < -2250) {
    encoderPos = 0;
  }
  if (PINB & 0b00000001) {
    encoderPos++;
  }
  else {
    encoderPos--;
  }
}

void pwmOut(float out) {
  if (out > 0) {
    analogWrite(LPWM, out);
    analogWrite(RPWM, 0);
  }
  else {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, abs(out));
  }
}

void loop() {
  pos = (encoderPos * 360) / 448;
  now = millis();
  int timeChange = (now - lastTime);

  if (timeChange >= 500) {
    temp = (360 * 1000 * (encoderPos - lastPos)) / (448 * (now - lastTime));

    if ((encoderPos < -2 || encoderPos > 2) && temp >= -60 && temp <= 60) {
      vel = temp;
      lastTime = now;
      lastPos = encoderPos;
    }
  }

  pwmOut(output);

  if ((now - lasttimepub) > 100) {
    joint_state.data_length = 2;
    joint_state.data[0] = pos;
    joint_state.data[1] = vel;
    pub.publish(&joint_state);
    lasttimepub = now;
  }
  nh.spinOnce();
}
