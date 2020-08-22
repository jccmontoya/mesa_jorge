/**
  @file esp32_cubo.ino

  @mainpage Main Page

  @section intro_sec Introduction

  This is the documentation for the cube device
  designed for the social robot Mini.

  @section author Author

  Written by Carlos Manuel Gomez Jimenez.

  @section license License

  BSD license, all text here must be included in any redistribution.

*/

#include "WiFi.h"
#include <ros.h>
#include <mesa_msgs/arduino_cmd.h>
#include <mesa_msgs/rgb_msg.h>

#include <NeoPixelBus.h>

//////////////////////
//    Parameters    //
//////////////////////

#define COLORSATURATION             128             ///< Maximum LED color value, out of 255
#define TOUCH_THRESOLD              40              ///< Greater the value, more the sensitivity
#define BATTERY_PIN                 A13             ///< Internal pin of ESP32, connected to battery through voltage divisor
touch_pad_t touchPin;                               ///< Not sure whats for

//////////////////////
//    Variables     //
//////////////////////

int i                             = 0;              ///< Incremental variable for LEDs
bool touch_bottom_detected        = false;          ///< Flag for touch pad of BOTTOM Face
bool touch_top_detected           = false;          ///< Flag for touch pad of TOP Face
bool touch_right_detected         = false;          ///< Flag for touch pad of RIGHT Face
bool touch_left_detected          = false;          ///< Flag for touch pad of LEFT Face
bool touch_front_detected         = false;          ///< Flag for touch pad of FRONT Face
bool touch_back_detected          = false;          ///< Flag for touch pad of BACK Face
bool touch_top_unreleased         = false;          ///< Flag for touch pad of TOP Face
const uint16_t LEDPixelCount      = 120;              ///< Number of total LEDs
const uint8_t PixelPin            = 25;             ///< Pin of LEDs connection to ESP32
unsigned long watchdog_previous   = 0;              ///< Last time that the sleep wachdog was reset
unsigned long watchdog_interval   = 60000;          ///< Time to wait before going to deep sleep
unsigned long timer10hz_previous  = 0;              ///< Last time that the 10 Hz timer was reset
unsigned long timer10hz_interval  = 100;            ///< Time to wait before doing the desired action
unsigned long timer0_1hz_previous = 0;              ///< Last time that the 0,1 Hz timer was reset
unsigned long timer0_1hz_interval = 10000;          ///< Time to wait before doing the desired action
unsigned long currentMillis;                        ///< Number of seconds since boot of ESP32

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(120, PixelPin);        ///< Object declaration of LED instances, called strip

//////////////////////
//Colors Definitions//
//////////////////////
/*
  RgbColor red(COLORSATURATION, 0, 0);
  RgbColor lightblue(0,0,5);
  RgbColor green(0, COLORSATURATION, 0);
  RgbColor blue(0, 0, COLORSATURATION);
  RgbColor white(COLesORSATURATION);
  RgbColor black(0);
*/
ros::NodeHandle_<ArduinoHardware,1,1,1000,1000> nh;

/**
   @brief Function executed at receiveing some message in the sleep topic.
   @param msg Description of the first parameter of the function.
   @return Describe what the function returns.
   @see https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   @note Something to note.
   @warning Warning.
*/
void ledCallback(const mesa_msgs::arduino_cmd& mensaje)
{
  digitalWrite(13, HIGH-digitalRead(13));   // turn the LED on (HIGH is the voltage level)
  for (int i = 0; i < 120; i++)
  {
    strip.SetPixelColor(i, RgbColor((int)mensaje.msg[i].r,(int)mensaje.msg[i].g,(int)mensaje.msg[i].b));
    if((mensaje.msg[i].r!=0)||(mensaje.msg[i].r!=0)||(mensaje.msg[i].r!=0))
    {
      nh.loginfo("pixel no vacio");
    }
  }
  strip.Show();
  digitalWrite(13, HIGH-digitalRead(13));   // turn the LED on (HIGH is the voltage level)

}


ros::Subscriber<mesa_msgs::arduino_cmd> led_sub("toggle_led", &ledCallback);            ///< Publisher of /cubo_n/sleep topic with Empty type message



/**
   @brief Setup function. Executed once at the beginning.
   @return None.
*/
void setup()
{
  nh.initNode();
  nh.subscribe(led_sub);
  pinMode(13, OUTPUT);
  strip.Begin();
  strip.Show();
    for (int i = 0; i < 50; i++)
  {
    strip.SetPixelColor(i, RgbColor(0,0,0));
  }
  strip.Show();
}

/**
   @brief Main loop. Executed repeatedly meanwhile the power persist.
   @return None.
   @note Something to note.
   @warning Warning.
*/
void loop()
{
  nh.spinOnce();
  delay(100);
}
