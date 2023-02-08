#!/usr/bin/env python3

import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage  
# from std_msgs.msg import Empty
from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse 

class ArucoDetectService():

    def __init__(self) -> None:
        self.image_msg = None
        self.service_server = rospy.Service('aruco_detect', ArucoDetect, self.service_request)   
        sub = rospy.Subscriber('/front_camera/image_raw/compressed', CompressedImage, self.image_cb)

        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def image_cb(self, msg: CompressedImage) -> None:
        self.image_msg =  msg   

    def service_request(self, request) -> ArucoDetectResponse:

        img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg, desired_encoding='passthrough')

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict, parameters=self.arucoParams)
        # print(corners, ids)
        if len(corners) > 0:
            data = []
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                size = cv2.contourArea(corners)

                if size > 500:                   
                    data.append((markerID, size)) 
                
            if len(data) >0:
                data = sorted(data, key=lambda x: x[1], reverse=True)              
                rospy.loginfo(f"ArucoService: Have marker result:  {data[0][0]}")
                return ArucoDetectResponse(id = data[0][0], size = int(data[0][1]))
  
        return ArucoDetectResponse(0,0)  

rospy.init_node('aruco_detect_server')

aruco = ArucoDetectService()
rospy.spin()

