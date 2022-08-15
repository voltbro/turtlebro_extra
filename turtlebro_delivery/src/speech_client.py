#!/usr/bin/env python3

import rospy, toml
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from pathlib import Path

class SpeechClient():

    def __init__(self) -> None:

        speech_config_file = rospy.get_param('~speech_config_file', str(
                Path(__file__).parent.absolute()) + '/../data/speech.toml')

        speech_config = toml.load(speech_config_file)
        rospy.loginfo(f"Loading speech file {speech_config_file}")

        self.text = speech_config['text']
        self.config = speech_config['config']

        if self.config['need_speech']:
            self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
            rospy.loginfo(f"Waiting for festival_speech service")
            self.speech_service.wait_for_service()
            rospy.loginfo(f"Have festival_speech service")  
        else:
            rospy.loginfo(f"Speech client not init")    

    def say(self, state):

        if state in self.text:
            if self.config['debug_text']:
                rospy.loginfo(f"SpeechClient: text: {self.text[state]}")             

            if self.config['need_speech']:      
                result: SpeechResponse = self.speech_service.call(SpeechRequest(data = self.text[state]))
        else :
            rospy.loginfo("SpeechClient: No state message")
            
if __name__ == '__main__':
    speech_client = SpeechClient()