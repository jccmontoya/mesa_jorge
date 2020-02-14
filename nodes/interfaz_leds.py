#!/usr/bin/env python2.7
import os
import rospy
from mesa_msgs.msg import rgb_msg
from mesa_msgs.msg import arduino_cmd
from mesa_msgs.msg import arduino_serialize_msg
import time
import sys
import rospkg
import string
import sys


class subypubs:
    def __init__(self):
        #self.pub = rospy.Publisher('/out_value', Int32, latch=True)
        self.command_sub =rospy.Subscriber('color_selected',arduino_cmd,self.leds_callback)
        self.leds_pub  =rospy.Publisher ('arduino_leds',arduino_serialize_msg,queue_size=10)
        self.mensaje=arduino_serialize_msg()
    def leds_callback(self,leds_value):
        for i in range(leds_value.TOTAL_LEDS):
        		self.mensaje.values=leds_value.msg[i]
        		self.mensaje.total=leds_value.TOTAL_LEDS
        		self.mensaje.current=i
        		self.mensaje.fade=leds_value.fade
        		self.leds_pub.publish(self.mensaje)
        		time.sleep(0.005)
        

          


if __name__ == '__main__':
    rospy.init_node('interfaz_leds')
    a=subypubs()
    rate=rospy.Rate(20)
    while not rospy.is_shutdown():
        #print "LUZ-PIR- -TOUCH-RELEASE-ENCODER-ENDSTOP"
        #print arr
        rate.sleep()
