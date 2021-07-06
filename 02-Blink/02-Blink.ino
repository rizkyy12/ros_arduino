#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13)); 
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
