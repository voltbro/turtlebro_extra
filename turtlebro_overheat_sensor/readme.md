
### Description

This package allows you to start a node for receiving data
from the AMG88xx GridEYE 8x8 IR camera thermal sensor.
 When a heat source with a temperature higher than
 threshold is detected, the detection node sends info message with
 custom message format HeatAlert to /heat_sensor_output topic.
 After that, the node continues work, but for the first 10 seconds it ignores all sources of heat.


### Package installation on RPI

Install the package on RaspberryPi in the "standard" way:

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_overheat_sensor.git
cd ~/catkin_ws
catkin_make --pkg turtlebro_overheat_sensor
```

### Connecting thermal sensor

AMG88xx thermal sensor   must be connected to built-in arduino compatible controller via I2C protocol. Wires from AMG88XX pins must be connected to one of the white connectors A8-A11 or A13-A15 on the Turtlebro Board.

sensor documentation:  
https://cdn.sparkfun.com/assets/4/1/c/0/1/Grid-EYE_Datasheet.pdf  
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-amg8833-8x8-thermal-camera-sensor.pdf  

Wiring:  

AMG88XX pins -> Turtlebro pin  
VIN -> 5V  
GND -> GND  
SDA -> SDA  
SCL -> SCL  

Example about connecting AMG88xx to Arduino:  
https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor/arduino-wiring-test  


### Connecting led lamp

Led lamp '+' pin must be connected STRICTLY to pin marked 'GPIO' in  white connector marked as 'A12'.   
Led lamp '-' pin must be connected to 'GND' pin on the same connector marked as 'A12'.  

NOTE:  
If you are using an unbranded LED lamp module, connect a current limiting resistor to the LED.  

### Connecting limit switch

Limit switch 'NC' pin must be connected STRICTLY to pin marked 'GPIO' in  white connector marked as 'A13'.   
Limit switch 'GND' pin must be connected to 'GND' pin on the same connector marked as 'A13'.  
Limit switch 'VCC' pin must be connected to '5V' pin on the same connector marked as 'A13'.  


### How to install on Arduino

Install Arduino Ide https://www.arduino.cc/en/main/software  
 - Open the Arduino Library Manager, find AMGXX library in search string, and install it. You also can download it directly from https://github.com/adafruit/Adafruit_AMG88xx  

 - Generate and copy ros_lib to your Arduino Libraries.

 - Open file src/arduino/amg88xx_main/amg88xx_main.ino from cloned repo in Arduino IDE.
Connect built-in turtlebro`s Arduino Mega via USB, and upload script to it.
(or upload remotely)

After uploading you must see topics "amg88xx_pixels" , "/alarm_led" and "/limit_switch" in list of ros topics.

### Launch

Launch only detector node:
```
roslaunch turtlebro_overheat_sensor heat_sensor.launch
```

Heat detector node ('heat_sensor') will read topic "amg88xx_pixels" (it publish array of 64 floats, those it got from sensor) and check maximum value of temperature from that array.
If maximum value is bigger than threshold (threshold can be set from .launch file), node sends
information string to topic '/heat_sensor_output'
For 10 seconds patrol node enters the standby state.

You can manually pub message to "/alarm_led" topic to set lamp on and off.  
To turn lamp on.  
```
rostopic pub /alarm_led std_msgs/Bool "data: true"   
```
To turn lamp off.  
```
rostopic pub /alarm_led std_msgs/Bool "data: false"   
```

You can manually read messages to "/amg88xx_pixels" topic to see array of temperature data directly from sensor  
```
rostopic echo /amg88xx_pixels  

```
You must see something like that:
```
---
layout:
  dim: []
  data_offset: 0
data: [18.5, 18.0, 18.5, 18.5, 18.5, 18.75, 18.75, 18.0, 18.75, 18.25, 18.5, 18.0, 18.5, 18.0, 18.5, 18.25, 18.75, 19.0, 18.5, 19.0, 18.75, 18.5, 18.75, 19.0, 19.0, 18.25, 18.75, 18.75, 18.75, 18.75, 18.5, 18.75, 18.75, 18.25, 19.0, 18.75, 18.75, 19.0, 19.0, 18.5, 18.0, 17.75, 18.75, 18.75, 18.25, 18.0, 18.5, 18.75, 18.0, 17.25, 18.0, 18.0, 18.25, 18.5, 18.75, 19.0, 18.25, 19.25, 19.0, 19.0, 18.25, 19.25, 19.5, 19.0]
---

```

You can manually read messages from "/limit_switch" topic to see state of   
```
rostopic echo /limit_switch  

```
You must see something like that:  
```
data: True
---
data: True
---
data: True
---
data: False
---
data: False
---
```
