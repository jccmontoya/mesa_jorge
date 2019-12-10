/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <mesa_msgs/button_msg.h>


ros::NodeHandle nh;

mesa_msgs::button_msg pushed_msg;
// std_msgs::Bool pushed_msg;
ros::Publisher pub_button("light_button_status", &pushed_msg);

const int button_pin = 2;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = ! digitalRead(button_pin);

  for(int i = 0 ; i<4; i++)
  {
    pushed_msg.buttons_pressed[i] = false;
  }
 
}

void loop()
{
  
  bool reading =  digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.buttons_pressed[0] = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
  
  nh.spinOnce();
}
