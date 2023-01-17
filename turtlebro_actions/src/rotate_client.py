#! /usr/bin/env python3
import sys
import rospy
import actionlib

from turtlebro_actions.msg import RotationAction, RotationGoal

class RotateClient(object):

    def __init__(self):
        
        rospy.loginfo("Run: %s", self.__class__)

        rospy.on_shutdown(self.shutdown)

        self.client = actionlib.SimpleActionClient('action_rotate', RotationAction)
        self.client.wait_for_server()
        
    def SendGoal(self, goal_val, speed_val):
        
        rotation_goal = RotationGoal(goal = goal_val, speed = speed_val)
        self.client.send_goal(rotation_goal, feedback_cb=self.action_feedback)

        self.client.wait_for_result()
        return self.client.get_result()

    def action_feedback(self, fb):
        rospy.loginfo("Current pose: %s", fb.feedback)

    def shutdown(self):
        rospy.loginfo("Rotate Client Node Shutdown")
        self.client.cancel_all_goals()

if __name__ == '__main__':
    try:

        rospy.init_node('rotate_client_node')

        client = RotateClient()
        result = client.SendGoal(10, 1.2)

        rospy.loginfo("Action Result pose: %s" , result)
        rospy.signal_shutdown("Stop and exit")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)