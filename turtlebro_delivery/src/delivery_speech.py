#!/usr/bin/env python3

from queue import Empty
import rospy, toml
from delivery import DeliveryRobot
from speech_client import SpeechClient
from config import delivery_config

class DeliverySpeechRobot(DeliveryRobot):
    def __init__(self, delivery_config: dict) -> None:
        self.speech_client = SpeechClient()
        super().__init__(delivery_config)

    def set_state(self, state: str):
        super().set_state(state)
        self.speech_client.say(state)

if __name__ == '__main__':
    try:
        rospy.init_node('delivery_speech_node')

        robot = DeliverySpeechRobot(
                    delivery_config = delivery_config)


        robot.spin()

    except rospy.ROSInterruptException:

        robot.on_shutdown()
        rospy.loginfo("Delivery stopped due to ROS interrupt")        