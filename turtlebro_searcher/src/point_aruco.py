#!/usr/bin/env python3

from turtlebro_patrol.srv import PatrolPointCallback, PatrolPointCallbackRequest, PatrolPointCallbackResponse
from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse, ArucoDetectRequest
from std_msgs.msg import String


import rospy
import toml
from pathlib import Path

rospy.init_node('searcher_aruco_service')

aruco_service = rospy.ServiceProxy('aruco_detect', ArucoDetect)
rospy.loginfo(f"Waiting for aruco service")
aruco_service.wait_for_service()
rospy.loginfo(f"Have aruco service")

pub = rospy.Publisher('aruco_marker', String, queue_size=10)

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

    text = "Location unknown"

    if point_name in text_data:
        text = text_data[point_name]

    if point_name != 'home':

        aruco_result: ArucoDetectResponse = aruco_service.call(
            ArucoDetectRequest())

        if aruco_result.id > 0:
            if str(aruco_result.id) in aruco_data:
                text += ". " + aruco_data[str(aruco_result.id)]['text']
            else:
                text += ". " + aruco_data['unknown']['text']
        else:
            text += ". " + aruco_data['empty']['text']

        pub.publish(text)

    return PatrolPointCallbackResponse(1, "Search end")


s = rospy.Service('point_aruco', PatrolPointCallback, handle_request)
rospy.loginfo("point_aruco Service Ready")
rospy.spin()
