#!/usr/bin/env python3

from turtlebro_patrol.srv import PatrolPointCallback, PatrolPointCallbackRequest, PatrolPointCallbackResponse
from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse, ArucoDetectRequest


import rospy
import toml
from pathlib import Path

rospy.init_node('excursion_aruco_service')

speech_service = rospy.ServiceProxy('festival_speech', Speech)
rospy.loginfo(f"Waiting for festival_speech service")
speech_service.wait_for_service()
rospy.loginfo(f"Have festival_speech service")

aruco_service = rospy.ServiceProxy('aruco_detect', ArucoDetect)
rospy.loginfo(f"Waiting for aruco service")
aruco_service.wait_for_service()
rospy.loginfo(f"Have aruco service")

patrol_data_file = rospy.get_param('~patrol_data_file', str(
    Path(__file__).parent.absolute()) + '/../data/goals.toml')
patrol_data = toml.load(patrol_data_file)
rospy.loginfo(f"Loading speech from file {patrol_data_file}")

text_data = {}
text_data['home'] = patrol_data['home']['text']
for patrol_point in patrol_data['patrolling']:
    text_data[patrol_point['name']] = patrol_point['text']

aruco_data_file = rospy.get_param('~aruco_data_file', str(
    Path(__file__).parent.absolute()) + '/../data/aruco_text.toml')
aruco_data = toml.load(aruco_data_file)

rospy.loginfo(f"Loading aruco file {aruco_data_file}")


def handle_request(req: PatrolPointCallbackRequest):

    point_name = req.patrol_point.name

    text = "Местоположение неизвестно"

    if point_name in text_data:
        text = text_data[point_name]

    if point_name != 'home':

        aruco_result: ArucoDetectResponse = aruco_service.call(
            ArucoDetectRequest())

        if aruco_result.id > 0:
            if str(aruco_result.id) in aruco_data:
                text += " " + aruco_data[str(aruco_result.id)]['text']
            else:
                text += " " + aruco_data['unknown']['text']
        else:
            text += " " + aruco_data['empty']['text']

    result: SpeechResponse = speech_service.call(SpeechRequest(text))

    return PatrolPointCallbackResponse(1, "Speech end")


s = rospy.Service('point_aruco', PatrolPointCallback, handle_request)
rospy.loginfo("point_aruco Service Ready")
rospy.spin()
