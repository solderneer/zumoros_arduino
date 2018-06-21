#define USBCON

#include <ros.h>
#include <Wire.h>
#include <Zumo32U4.h>

#include <std_msgs/Int16.h>

// Function Prototypes
void lmotorCb(const std_msgs::Int16 &lmotor_msg);
void rmotorCb(const std_msgs::Int16 &rmotor_msg);

// Zumo Object Initializations
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

// ROS Initializations
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> lmotor_sub("lmotor", &lmotorCb);
ros::Subscriber<std_msgs::Int16> rmotor_sub("rmotor", &rmotorCb);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}

void lmotorCb(const std_msgs::Int16 &lmotor_msg) {
  // Do smth in the callback
  int16_t leftSpeed = constrain(lmotor_msg.data, -400, 400);
  motors.setLeftSpeed(leftSpeed);
}

void rmotorCb(const std_msgs::Int16 &rmotor_msg) {
  // Do smth in the callback
  int16_t rightSpeed = constrain(rmotor_msg.data, -400, 400);
  motors.setRightSpeed(rightSpeed);
}

