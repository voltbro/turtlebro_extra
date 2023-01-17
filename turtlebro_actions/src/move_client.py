#! /usr/bin/env python3
import sys
import rospy
import actionlib

from turtlebro_actions.msg import MoveAction, MoveGoal

class MoveClient(object):

    def __init__(self):

        rospy.loginfo("Run: %s", self.__class__)

        self.client = actionlib.SimpleActionClient('action_move', MoveAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown)
        
    def SendGoal(self, goal_val, speed_val):

        goal = MoveGoal(goal = goal_val, speed = speed_val)
        self.client.send_goal(goal, feedback_cb=self.action_feedback)

        self.client.wait_for_result()
        return self.client.get_result()

    def action_feedback(self, fb):
        rospy.loginfo("Current pose: %s", fb.feedback)

    def shutdown(self):
        rospy.loginfo("Move Client Node Shutdown")
        self.client.cancel_all_goals()

if __name__ == '__main__':
    try:

        rospy.init_node('move_client_node')
        client = MoveClient()

        result = client.SendGoal(0.5, 0.22)

        rospy.loginfo("Action Result pose: %s" , result)
        rospy.signal_shutdown("Stop and exit")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)