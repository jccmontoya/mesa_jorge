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
from std_msgs.msg import Bool
from mesa_msgs.msg import button_msg as BM, color_selected as CS, light_selected as LS, audio_button as AB
import collections


class ButtonManager:
    def __init__(self):
        print("Initializing button_manager")
        rospy.Subscriber('light_button_status', BM, self.light_button_callback)
        rospy.Subscriber('audio_button_status', BM, self.audio_button_callback)
        self.color_pub = rospy.Publisher('color_selected', CS, queue_size=10)
        self.mode = 1
        self.audio_pub = rospy.Publisher('audio_button',AB, queue_size=10)
        print("button_manager initialized")

    def audio_button_callback(self,sound_buttons):
        sound_index = self.button_2_song_index(sound_buttons)
        ab_message = AB()
        ab_message.button_code = sound_index
        ab_message.mode = self.mode
        self.audio_pub.publish(ab_message)

    def light_button_callback(self, light_buttons):
        count = 0
        #light_buttons = data.buttons_pressed[:4]
        #sound_buttons = data.buttons_pressed[4:]
        index = 0
        count_light = self.count_true(light_buttons)
        buttons = []
        while index < len(light_buttons) and count < 2:
             if light_buttons[index]:
                 buttons.append(BUTTONS[index])
                 count = count +1
        #         red = max(red,BUTTONS[index][0])
        #         green = max(green, BUTTONS[index][1])
        #         blue = max(blue, BUTTONS[index][2])
             index = index + 1
        # print([red, green, blue])
        color = self.mix_color(buttons, count)
        print(color)
        color_sel = CS()
        color_sel.color=color
        self.color_pub.publish(color_sel)

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


