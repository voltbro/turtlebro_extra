#!/usr/bin/env python3

from turtlebro_patrol.srv import PatrolPointCallback, PatrolPointCallbackRequest, PatrolPointCallbackResponse 
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest

import rospy
import toml
from pathlib import Path

rospy.init_node('excursion_point_service')

speech_service = rospy.ServiceProxy('festival_speech', Speech)
rospy.loginfo(f"Waiting for festival_speech service")
speech_service.wait_for_service()
rospy.loginfo(f"Have festival_speech service") 

patrol_data_file = rospy.get_param('~patrol_data_file', str(
        Path(__file__).parent.absolute()) + '/../data/goals.toml')
patrol_data = toml.load(patrol_data_file)
rospy.loginfo(f"Loading speech from file {patrol_data_file}")

text_data = {}
text_data['home'] = patrol_data['home']['text']
for patrol_point in patrol_data['patrolling']:
    text_data[patrol_point['name']] = patrol_point['text']

rospy.loginfo(f"Have speech text: {text_data}")

def handle_request(req:PatrolPointCallbackRequest):

    point_name = req.patrol_point.name
    
    text = "Местоположение неизвестно"
    
    if point_name in text_data:
        text = text_data[point_name]

    result: SpeechResponse = speech_service.call(SpeechRequest(text))

    return PatrolPointCallbackResponse(1, "Speech end")
 
s = rospy.Service('point_speech', PatrolPointCallback, handle_request)
rospy.loginfo("point_speech Service Ready")
rospy.spin()
