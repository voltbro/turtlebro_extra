#!/usr/bin/env python3

import rospy
from delivery import DeliveryRobot
from speech_client import SpeechClient


class DeliverySpeechRobot(DeliveryRobot):
    def __init__(self) -> None:
        self.speech_client = SpeechClient()
        super().__init__()

    def set_state(self, state: str):
        super().set_state(state)
        self.speech_client.say(state)


if __name__ == '__main__':
    try:
        rospy.init_node('delivery_speech_node')

        robot = DeliverySpeechRobot()
        robot.spin()

    except rospy.ROSInterruptException:

        robot.on_shutdown()
        rospy.loginfo("Delivery stopped due to ROS interrupt")
