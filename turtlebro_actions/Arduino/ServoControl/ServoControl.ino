/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <ros.h>
#include <Servo.h> 
#include <std_msgs/UInt16.h>

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};

ros::NodeHandle_<NewHardware>  nh;


Servo servo44;
Servo servo45;

void servo44_cb( const std_msgs::UInt16& cmd_msg){
  servo44.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void servo45_cb( const std_msgs::UInt16& cmd_msg){
  servo44.write(cmd_msg.data); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::UInt16> sub44("servo44", servo44_cb);
ros::Subscriber<std_msgs::UInt16> sub45("servo45", servo45_cb);



void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub44);
  nh.subscribe(sub45);
  
  servo44.attach(44); //attach it to pin 9
  servo45.attach(45); //attach it to pin 9
  delay(50);
  servo44.write(90);
  servo45.write(90);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}