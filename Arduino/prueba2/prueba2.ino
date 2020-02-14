#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <NeoPixelBus.h>
#include <mesa_msgs/arduino_serialize_msg.h>

#define colorSaturation 128


ros::NodeHandle  nh;

bool touch1detected = false;
bool touch2detected = false;
const uint16_t PixelCount = 144; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 25;  // make sure to set this to the correct pin, ignored for Esp8266

// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor red(colorSaturation, 0, 0);
RgbColor lightblue(0,0,5);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

HslColor hslRed(red);
HslColor hslGreen(green);
HslColor hslBlue(blue);
HslColor hslWhite(white);
HslColor hslBlack(black);


int i;

void turnonCallback(const std_msgs::Empty& msg) {
  for(int i=0;i<PixelCount;i++){
       strip.SetPixelColor(i,red);
     }
     strip.Show();
}

void turnoffCallback(const std_msgs::Empty& msg) {
  for(int i=0;i<PixelCount;i++){
       strip.SetPixelColor(i,black);
     }
     strip.Show();
}

void messageCb( const mesa_msgs::arduino_serialize_msg& message){
  strip.SetPixelColor(message.current,RgbColor(message.values.r,message.values.g,message.values.b));
  if(message.current==message.total-1)
    strip.Show();
}


ros::Subscriber<std_msgs::Empty> sub("turn_on", &turnonCallback);
ros::Subscriber<std_msgs::Empty> sub2("turn_off", &turnoffCallback);
ros::Subscriber<mesa_msgs::arduino_serialize_msg> sub3("arduino_leds", &messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  strip.Begin();
  strip.Show();
  
}

void loop() {
  nh.spinOnce();
  delay(100);
}
