#!/usr/bin/env python3

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback

class FakeMoveBase():

    def __init__(self):

        rospy.init_node('fake_move_base')
        self.result = MoveBaseActionResult()
        self.feedback = MoveBaseActionFeedback()
        self.fmb = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute_cb)

  
    def execute_cb(self, goal):
        rospy.loginfo("Have new goal")
        success = True

        for i in range(0, 10):

            if self.fmb.is_preempt_requested():
                rospy.loginfo('Action Preempted')
                self.fmb.set_preempted()
                success = False
                break
            rospy.sleep(0.5)
            

        self.fmb.publish_feedback(self.feedback.feedback)

        if success:
            self.result = MoveBaseActionResult()
            rospy.loginfo('Succeeded')
            self.fmb.set_succeeded(self.result)

server = FakeMoveBase()
rospy.spin()