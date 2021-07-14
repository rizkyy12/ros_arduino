#include <ros.h>
#include <rospy_tutorials/Floats.h>

int now;

ros::NodeHandle nh;
rospy_tutorials::Floats data_floats;
ros::Publisher pub("/data_arduino", &data_floats);

void setup() {
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  now = millis();
  for (int i = 0; i <= 100 ; i++) {
    data_floats.data_length = 1;
    data_floats.data[0] = i;
    pub.publish(&data_floats);
    delay(100);    
  }
  nh.spinOnce();
}
