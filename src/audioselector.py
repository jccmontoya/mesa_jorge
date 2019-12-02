import rospy
from mesa_msgs.msg import audio_button as AB, sound_selected as SS

class AudioSelector:
    def __init__(self):
        print("Initializing audio_selector")

        rospy.Subscriber('audio_button', SS, self.audio_selection_callback)
        print("audio_selector correctly initialized")

    def audio_selection_callback(self,data):
        pass

if __name__ == '__main__':
    rospy.init_node("audio_manager")
    node = AudioSelector()
    rospy.spin()