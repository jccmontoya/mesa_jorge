#!/usr/bin/env python3

import rospy
import rospkg
import os
import random
from mesa_msgs.msg import audio_button as AB, sound_selected as SS


class AudioSelector:
    def __init__(self):
        print("Initializing audio_selector")
        rospy.Subscriber('audio_button', AB, self.audio_selection_callback)
        self.audio_selected_pub = rospy.Publisher('audio_selection', SS, queue_size=10)
        rospack = rospkg.RosPack()
        self.mode = 1
        self.audio_path = rospack.get_path('mesa_jorge') + '/data/mp3/'

        self.air_path = self.audio_path + 'music/air/'
        self.water_path = self.audio_path + 'music/water/'
        self.fire_path = self.audio_path + 'music/fire/'
        self.earth_path = self.audio_path + 'music/earth/'

        self.sound_air_path = self.audio_path + 'sound/air/'
        self.sound_water_path = self.audio_path + 'sound/water/'
        self.sound_fire_path = self.audio_path + 'sound/fire/'
        self.sound_earth_path = self.audio_path + 'sound/earth/'

        self.air_list = []
        self.water_list = []
        self.fire_list = []
        self.earth_list = []
        self.air_len = -1
        self.water_len = -1
        self.fire_len = -1
        self.earth_len = -1

        self.sound_air_list = []
        self.sound_water_list = []
        self.sound_fire_list = []
        self.sound_earth_list = []
        self.sound_air_len = -1
        self.sound_water_len = -1
        self.sound_fire_len = -1
        self.sound_earth_len = -1

        self.get_audio_files()
        print("audio_selector correctly initialized")

    def get_audio_files(self):
        self.air_list = os.listdir(self.air_path)
        self.water_list = os.listdir(self.water_path)
        self.fire_list = os.listdir(self.fire_path)
        self.earth_list = os.listdir(self.earth_path)
        self.air_len = len(self.air_list)
        self.water_len = len(self.water_list)
        self.fire_len = len(self.fire_list)
        self.earth_len = len(self.earth_list)
        self.sound_air_list = os.listdir(self.sound_air_path)
        self.sound_water_list = os.listdir(self.sound_water_path)
        self.sound_fire_list = os.listdir(self.sound_fire_path)
        self.sound_earth_list = os.listdir(self.sound_earth_path)
        self.sound_air_len = len(self.sound_air_list)
        self.sound_water_len = len(self.sound_water_list)
        self.sound_fire_len = len(self.sound_fire_list)
        self.sound_earth_len = len(self.sound_earth_list)

    def audio_selection_callback(self, data):
        audio_msg = SS()
        audio_msg.sound_selected = ''
        audio_msg.category = ''

        if self.mode == 0:
            audio_msg = self.select_music(data)
        elif self.mode == 1:
            audio_msg = self.select_sound(data)

        if audio_msg.sound_selected != '':
            print('Audio selected: ', audio_msg.sound_selected)
            print('Publishing audio_selected')
            self.audio_selected_pub.publish(audio_msg)

    def select_sound(self, data):
        audio_selected = ''
        category = ''

        if data.button_code == 1 and self.sound_fire_len > 0:  # Fire sound
            audio_selected = self.sound_fire_path + self.sound_fire_list[random.randrange(self.sound_fire_len)]
            category = 'fire'
        elif data.button_code == 2 and self.sound_earth_len > 0:  # Earth sound
            audio_selected = self.sound_earth_path + self.sound_earth_list[random.randrange(self.sound_earth_len)]
            category = 'earth'
        elif data.button_code == 4 and self.sound_water_len > 0:  # Water sound
            audio_selected = self.sound_water_path + self.sound_water_list[random.randrange(self.sound_water_len)]
            category = 'water'
        elif data.button_code == 8 and self.sound_air_len > 0:  # Air sound
            audio_selected = self.sound_air_path + self.sound_air_list[random.randrange(self.sound_air_len)]
            category = 'air'

        audio_msg = SS()
        audio_msg.sound_selected = audio_selected
        audio_msg.category = category
        return audio_msg

    def select_music(self, data):
        audio_selected = ''
        category = ''

        if data.button_code == 1 and self.fire_len > 0:  # Fire music
            audio_selected = self.fire_path + self.fire_list[random.randrange(self.fire_len)]
            category = 'fire'
        elif data.button_code == 2 and self.earth_len > 0:  # Earth music
            audio_selected = self.earth_path + self.earth_list[random.randrange(self.earth_len)]
            category = 'earth'
        elif data.button_code == 4 and self.water_len > 0:  # Water music
            audio_selected = self.water_path + self.water_list[random.randrange(self.water_len)]
            category = 'water'
        elif data.button_code == 8 and self.air_len > 0:  # Air music
            audio_selected = self.air_path + self.air_list[random.randrange(self.air_len)]
            category = 'air'

        audio_msg = SS()
        audio_msg.sound_selected = audio_selected
        audio_msg.category = category
        return audio_msg


if __name__ == '__main__':
    rospy.init_node("audio_selector")
    node = AudioSelector()
    rospy.spin()
