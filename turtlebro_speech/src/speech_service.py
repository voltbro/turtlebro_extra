#!/usr/bin/env python3

import rospy
import subprocess

from std_msgs.msg import String
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest 

class FestivalSpeechService():

    def __init__(self) -> None:
        rospy.loginfo("Start festivel TTS service")
        self.service_server = rospy.Service('festival_speech', Speech, self.service_request)   

    def service_request(self, request:SpeechRequest) -> SpeechResponse:
        rospy.loginfo(f"Start speech: {request.data}")
        # subprocess.call('echo '+text+'|festival --tts --language russian', shell=True)
        subprocess.call(f'echo \'(SayText "{request.data}")\'| festival_client', shell=True)
        rospy.loginfo("Speech end")
  
        return SpeechResponse(True)  

rospy.init_node('festival_speech_server')

aruco = FestivalSpeechService()
rospy.spin()

