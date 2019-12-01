import rospy
from std_msgs.msg import Empty
from mesa_msgs.msg import sound_selected as SS
import vlc


class audio_manager:
    def __init__(self):
        print("Initializing button_manager")
        self.isPaused = True
        self.loaded_track = False
        self.player = vlc.MediaPlayer()
        rospy.Subscriber('audio_selection', SS, self.audio_selection_callback)
        rospy.Subscriber('pause_playback',Empty,self.pause_playback)
        rospy.Subscriber('stop_playback',Empty,self.stop_playback)

    def audio_selection_callback(self,data):
        if not self.isPaused:
            self.player.stop()
            self.isPaused == True
        self.player = vlc.MediaPlayer(data.sound_selected)
        self.loaded_track = True
        self.isPaused = False

    def pause_playback(self):
        if not self.isPaused:
            self.player.pause()
            self.isPaused = True
        elif self.loaded_track:
            self.player.play()
            self.isPaused = False

    def stop_playback(self):
        if self.loaded_track:
            self.player.stop()
            self.loaded_track = False
            self.isPaused = True


if __name__ == '__main__':
    rospy.init_node("audio_manager")
    node = audio_manager()
    rospy.spin()