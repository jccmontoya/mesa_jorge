#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from mesa_msgs.msg import sound_selected as SS
import vlc


class AudioManager:
    def __init__(self):
        print("Initializing audio_manager")
        self.isPaused = True
        self.loaded_track = False
        self.player = vlc.MediaPlayer()
        rospy.Subscriber('audio_selection', SS, self.audio_selection_callback)
        rospy.Subscriber('pause_playback', Empty, self.pause_playback)
        rospy.Subscriber('stop_playback', Empty, self.stop_playback)
        self.last_message = SS()
        self.last_message.sound_selected = ''
        self.last_message.category = ''

        print("audio_manager correctly initialized")

    def audio_selection_callback(self, data):
        if (self.player.get_state() == vlc.State.Playing or self.player.get_state() == vlc.State.Paused) and \
                data.category == self.last_message.category:
            self.pause_playback('')
        else:
            if not self.isPaused:
                self.player.stop()
                self.isPaused = True
            try:
                self.player.release()
                self.player = vlc.MediaPlayer(data.sound_selected)
                self.loaded_track = True
                self.isPaused = False
                self.player.play()
                self.last_message.sound_selected = data.sound_selected
                self.last_message.category = data.category
                print('Playing sound ', data.sound_selected)
            except:
                rospy.logerr('File not found: %s', data.sound_selected)

    def pause_playback(self, data):
        if not self.isPaused:
            self.player.pause()
            self.isPaused = True
            print('Player paused')
        elif self.loaded_track:
            self.player.play()
            self.isPaused = False
            print('Playback resumed')

    def stop_playback(self, data):
        if self.loaded_track:
            self.player.stop()
            self.loaded_track = False
            self.isPaused = True
            self.player = vlc.MediaPlayer()
            print('Playback stopped')


if __name__ == '__main__':
    rospy.init_node("audio_manager")
    node = AudioManager()
    rospy.spin()
