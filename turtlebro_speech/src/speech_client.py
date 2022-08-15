#!/usr/bin/env python3

import rospy

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest 


class SpeechClient():

    def __init__(self) -> None:

        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
        rospy.loginfo(f"Waiting for festival_speech service")
        self.speech_service.wait_for_service()
        rospy.loginfo(f"Have festival_speech service")  

    def say(self, text):

        result: SpeechResponse = self.speech_service.call(SpeechRequest(data = text))

rospy.init_node('festival_speech_client')

speech = SpeechClient()
global_state("Привет мир")

rospy.sleep(2)