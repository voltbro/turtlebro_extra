#include <ros.h>
#include <std_msgs/Int16.h>
#include <FastLED.h>

#define DATA_PIN 30
#define NUM_LEDS 24
#define BRIGHTNESS 200

std_msgs::Int16 button_num;

CRGB leds[NUM_LEDS];

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};

void led_cb(const std_msgs::Int16 &led_msg){

// "red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6
  int pincolorred;
  int pincolorgreen;
  int pincolorblue;
 if(led_msg.data == 1)
 {
  pincolorred = 255;
  pincolorgreen = 0;
  pincolorblue = 0;
 }
 else if (led_msg.data == 2)
  {
  pincolorred = 0;
  pincolorgreen = 255;
  pincolorblue = 0;
 }
  else if (led_msg.data == 3)
  {
  pincolorred = 0;
  pincolorgreen = 0;
  pincolorblue = 255;
 }
  else if (led_msg.data == 4)
  {
  pincolorred = 255;
  pincolorgreen = 255;
  pincolorblue = 0;
 }
  else if (led_msg.data == 5)
  {
  pincolorred = 255;
  pincolorgreen = 255;
  pincolorblue = 255;
 }
  else if (led_msg.data == 6)
  {
  pincolorred = 0;
  pincolorgreen = 0;
  pincolorblue = 0;
 }
   else
  {
  pincolorred = 0;
  pincolorgreen = 0;
  pincolorblue = 0;
 }

 for(int i=0; i<24; i++)
 {
  leds[23-i].r = pincolorred;
  leds[23-i].g = pincolorgreen;
  leds[23-i].b = pincolorblue;
  FastLED.show();
 }  
}

ros::NodeHandle_<NewHardware>  nh;
ros::Publisher pub("/buttons", &button_num);
ros::Subscriber<std_msgs::Int16> sub("py_leds", &led_cb);

void setup() {
  delay(500); // sanity delay
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );
  pinMode(13, OUTPUT);
  pinMode(23, INPUT);
  pinMode(24, INPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop() {
  button_num.data = 0;
  if(digitalRead(23))
  {
    button_num.data = 23;
  }
  if(digitalRead(24))
  {
    button_num.data = 24;
  }
  pub.publish(&button_num);
  nh.spinOnce();
  delay(50);
}