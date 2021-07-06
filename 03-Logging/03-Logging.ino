#include <ros.h>

ros::NodeHandle nh;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!nh.connected()){
      nh.spinOnce();
    }
  nh.logdebug("Debug statement");
  nh.loginfo("Program info");
  nh.logwarn("Warnings.");
  nh.logerror("Errors..");
  nh.logfatal("Fatalities!");
  delay(5000);
}
