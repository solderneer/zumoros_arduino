#define USBCON

#include <ros.h>
#include <Wire.h>
#include <Zumo32U4.h>

#include <std_msgs/Int16.h>
#include <std_msgs/UInt8MultiArray.h>

// Function Prototypes
void lmotorCb(const std_msgs::Int16 &lmotor_msg);
void rmotorCb(const std_msgs::Int16 &rmotor_msg);

// Zumo Object Initializations
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

// ROS message declarations
std_msgs::UInt8MultiArray prox_data;

// ROS Initializations
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> lmotor_sub("lmotor", &lmotorCb);
ros::Subscriber<std_msgs::Int16> rmotor_sub("rmotor", &rmotorCb);
ros::Publisher prox_pub("prox_sensor", &prox_data);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.advertise(prox_pub);

  proxSensors.initThreeSensors();

  // Setup the data structure for array
  prox_data.layout.data_offset = 0;
  prox_data.layout.dim[0].label = "prox_sensors";
  prox_data.layout.dim[0].size = 6;
  prox_data.layout.dim[0].stride = 1;
}

void loop() {
  static uint16_t lastSampleTime = 0;

  if((uint16_t)(millis() - lastSampleTime) >= 100) {
    lastSampleTime = millis();
    readproxSensors();
    prox_pub.publish(&prox_data);
  }
  
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

void readproxSensors(void) {
  proxSensors.read();
  prox_data.data[0] = proxSensors.countsLeftWithLeftLeds();
  prox_data.data[1] = proxSensors.countsLeftWithRightLeds();
  prox_data.data[2] = proxSensors.countsFrontWithLeftLeds();
  prox_data.data[3] = proxSensors.countsFrontWithRightLeds();
  prox_data.data[4] = proxSensors.countsRightWithLeftLeds();
  prox_data.data[5] =  proxSensors.countsRightWithRightLeds();
}

