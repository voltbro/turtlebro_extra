#!/usr/bin/env python3

import rospy, toml
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest


class SpeechClient():

    def __init__(self, params) -> None:


        self.text = params['text']
        self.config = params['config']

        if self.config['need_speech']:
            self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
            rospy.loginfo(f"Waiting for festival_speech service")
            self.speech_service.wait_for_service()
            rospy.loginfo(f"Have festival_speech service")  
        else:
            rospy.loginfo(f"Speech client not init")    

    def say(self, msg):

        if msg in self.text:
            text = self.text[msg]
        else :
            text = "Ошибка сообщения"  

        if self.config['debug_text']:
            rospy.loginfo(f"Speech text: {text}")             

        if self.config['need_speech']:      
            result: SpeechResponse = self.speech_service.call(SpeechRequest(data = text))

            
if __name__ == '__main__':

    params = toml.load('../data/speech.toml')
    speech_client = SpeechClient(params)