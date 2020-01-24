#!/usr/bin/env python3

RED     = [255, 0, 0]
GREEN   = [0, 255, 0]
BLUE    = [0, 0, 255]
YELLOW  = [255, 255, 0]
WHITE   = [255, 255, 255]
BLACK   = [0, 0, 0]
BROWN   = [128,60,0]
PURPLE  = [255, 0, 255]
ORANGE  = [255, 128, 0]
TURQUOISE = [0, 255, 255]

BUTTONS = [RED, GREEN, BLUE, YELLOW]

import rospy
from mesa_msgs.msg import button_msg as BM, arduino_cmd as ACMD, audio_button as AB, volume_cmd as VC
import collections
import os


class ButtonManager:
    def __init__(self):
        print("Initializing button_manager")
        rospy.Subscriber('light_button_status', BM, self.light_button_callback)
        rospy.Subscriber('audio_button_status', BM, self.audio_button_callback)
        rospy.Subscriber('volume_control', VC, self.volume_control_callback)
        self.color_pub = rospy.Publisher('color_selected', ACMD, queue_size=10)
        self.mode = 1
        self.audio_pub = rospy.Publisher('audio_button', AB, queue_size=10)
        print("button_manager initialized")

    def volume_control_callback(self, volume):
        vol_str = 'amixer -D pulse sset Master ' + str(volume.volume_rate) + '%'
        print(vol_str)
        os.system(vol_str)

    def audio_button_callback(self, sound_buttons):
        sound_index = self.button_2_song_index(sound_buttons.buttons)
        ab_message = AB()
        ab_message.button_code = sound_index
        ab_message.mode = self.mode
        print('Audio button received')
        print(sound_buttons.buttons)
        print('Sound index: ', sound_index)
        print('Publishing Audio Button')
        self.audio_pub.publish(ab_message)

    def light_button_callback(self, data):
        count = 0
        light_buttons = data.buttons
        index = 0
        buttons = []
        while index < len(light_buttons) and count < 2:
             if light_buttons[index]:
                 buttons.append(BUTTONS[index])
                 count = count +1
             index = index + 1
        color = self.mix_color(buttons, count)
        print('Light button received')
        print(data.buttons)
        print('Color: ', color)
        arduino_cmd = ACMD()
        for i in arduino_cmd.msg:
            i.r = color[0]
            i.g = color[1]
            i.b = color[2]
        self.color_pub.publish(arduino_cmd)

    def count_true(self,list):
        counter = collections.Counter(list)
        return counter[True]

    def mix_color(self,buttons, count):
        color = WHITE
        if count==1:
            return buttons[0]
        else:
            if buttons[0] == RED:
                if buttons[1] == GREEN:
                    color = BROWN
                elif buttons[1] == BLUE:
                    color = PURPLE
                elif buttons[1] == YELLOW:
                    color = ORANGE
            elif buttons[0] == GREEN:
                if buttons[1] == BLUE:
                    color = TURQUOISE
                elif buttons[1] == YELLOW:
                    color = [128, 255, 0]
            else:
                color = GREEN
        return color

    def button_2_song_index(self,data):
        acc = 1
        value = 0
        for x in data:
            if x:
                value = value + acc
            acc = acc * 2
        return value


if __name__ == '__main__':
    rospy.init_node("button_manager")
    node = ButtonManager()
    rospy.spin()


