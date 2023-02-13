/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch tries to read the pixels from the sensor

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

// prepare signal led output
// led connected to pin D30
#define ledPin A12
#define buttonPin A13

// timing for read AMG88xx data
int amg88xx_delay  = 1000; //ms
int sub_delay = 10; //ms
unsigned long previousMillis = 0; 

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];


class NewHardware : public ArduinoHardware
{
  public:
  // especially for using in turtlebro`s mainboard arduino
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};
// declare node
ros::NodeHandle_<NewHardware>  nh;

// declare msgs
std_msgs::Float32MultiArray f_array_msg;
std_msgs::Bool toggle_msg;
std_msgs::Bool button_msg;

// pub
ros::Publisher amg88xx_pixels_pub("amg88xx_pixels", &f_array_msg);
ros::Publisher button_state_pub("limit_switch", &button_msg);

void messageCb(const std_msgs::Bool& toggle_msg)
{
    digitalWrite(ledPin, toggle_msg.data);   // blink the led
}

// sub
ros::Subscriber<std_msgs::Bool> sub("alarm_led", &messageCb );


void setup()
{

    bool status;
    
    // led connected to pin D30 and to GND
    // so we have to digitalWrite(30, HIGH) to set on
    // and digitalWrite(30, LOW) to set off
    // default HIGH
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    pinMode(buttonPin, INPUT);
    // digitalWrite(ledPin, LOW);

    // default settings
    status = amg.begin();
    if (!status) {
        while (1);
    }
    delay(100); 

    f_array_msg.data = (float*)malloc(sizeof(float) * AMG88xx_PIXEL_ARRAY_SIZE);
    f_array_msg.data_length = AMG88xx_PIXEL_ARRAY_SIZE;

    nh.initNode();
    nh.advertise(amg88xx_pixels_pub);
    nh.advertise(button_state_pub);
    nh.subscribe(sub);

}

void loop()
{

    // check sub every 10 ms, but pub pixel array only every 1 sec

    unsigned long currentMillis = millis();
    int button_state = LOW;

    if (currentMillis - previousMillis >= amg88xx_delay)
    {
        previousMillis = currentMillis;
        amg.readPixels(pixels);

        for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; ++i)
        {
            f_array_msg.data[i] = pixels[i];
        }

        amg88xx_pixels_pub.publish(&f_array_msg);

        // although publish current button state
        button_state = digitalRead(buttonPin);
        if(button_state == LOW)
        {
            button_msg.data = false;
            button_state_pub.publish(&button_msg);
        }
        else
        {
            button_msg.data = true;
            button_state_pub.publish(&button_msg);
        }
    }

    nh.spinOnce();
    delay(sub_delay);
    
}
