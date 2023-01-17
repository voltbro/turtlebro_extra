#! /usr/bin/env python3
import sys
import rospy
import time

from std_msgs.msg import UInt16

class ServoClient(object):

    def __init__(self, servo):

        rospy.loginfo("Run: %s for servo %i", self.__class__, servo)

        rospy.on_shutdown(self.shutdown)

        self.servo = servo
        self.pub = rospy.Publisher("/servo" + str(servo), UInt16, queue_size=5)

    def move_await(self, angle):
        rospy.loginfo("Move servo %s to %i", self.servo, angle)
        
        while self.pub.get_num_connections() < 1:
            rospy.sleep(0.2)

        self.pub.publish(angle)
        rospy.sleep(1)   

    def move(self, angle):
        rospy.loginfo("Move servo %s to %i", self.servo, angle)
       
        self.pub.publish(angle)
        rospy.sleep(1)    


    def shutdown(self):
        rospy.loginfo("Servo %s Node Shutdown", self.servo)
        self.pub.unregister()
  
if __name__ == '__main__':   

    rospy.init_node('node_name')
    servo = ServoClient(44)

    print(servo.move_await(int(sys.argv[1])))
