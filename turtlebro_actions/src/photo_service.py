#! /usr/bin/env python3

import rospy

from turtlebro_actions.srv import Photo, PhotoResponse, PhotoRequest
from sensor_msgs.msg import CompressedImage

def handle_photo(req: PhotoRequest):
    photo_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
    return photo_msg

def photo_server():
    rospy.init_node('photo_service')
    s = rospy.Service('get_photo', Photo, handle_photo)
    rospy.loginfo("Start photos Service")
    rospy.spin()

if __name__ == "__main__":
    photo_server()