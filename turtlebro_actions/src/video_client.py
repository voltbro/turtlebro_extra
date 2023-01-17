#! /usr/bin/env python3
import sys
import rospy

from sensor_msgs.msg import CompressedImage

class VideoClient(object):

    def __init__(self):
        rospy.loginfo("Run: %s", self.__class__)

    def getImage(self):

        msg = rospy.wait_for_message("/front_camera/compressed", CompressedImage)
        return msg.data    

if __name__ == '__main__':   

    rospy.init_node('node_name')
    client = VideoClient()
    print(client.getImage())
