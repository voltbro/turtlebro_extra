#include <ros.h>


//buttons_part
#include <std_msgs/Int16.h>
std_msgs::Int16 button_num;

//LED part
#include <FastLED.h>
#define DATA_PIN 30
#define NUM_LEDS 24
#define BRIGHTNESS 200
CRGB leds[NUM_LEDS];

//Thermovisor part
#include <std_msgs/Float32MultiArray.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
std_msgs::Float32MultiArray thermo_arr;


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
ros::Publisher button_pub("/buttons", &button_num);
ros::Subscriber<std_msgs::Int16> sub("py_leds", &led_cb);
ros::Publisher thermo_pub("/thermovisor", &thermo_arr);

int read_buttons()
{
  if(digitalRead(23))
  {
    return 23;
  }
  if(digitalRead(24))
  {
    return 24;
  }
  else return 0;
}

void thermo_read()
{
  
}


void setup() {
  delay(500); // sanity delay
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );
  pinMode(13, OUTPUT);
  pinMode(23, INPUT);
  pinMode(24, INPUT);

  amg.begin(); //thermovisor
  thermo_arr.data = (float*)malloc(sizeof(float) * AMG88xx_PIXEL_ARRAY_SIZE);
  thermo_arr.data_length = AMG88xx_PIXEL_ARRAY_SIZE;
  
  nh.initNode();
  nh.advertise(button_pub);
  nh.advertise(thermo_pub);
  nh.subscribe(sub);
}

void loop() {
  amg.readPixels(pixels);
  for(int i = 0;i < AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {
    thermo_arr.data[i] = pixels[i];
  }
  thermo_pub.publish(&thermo_arr);
  
  //button part
  button_num.data = 0;
  button_num.data = read_buttons();
  button_pub.publish(&button_num);
  nh.spinOnce();
  delay(50);
}
