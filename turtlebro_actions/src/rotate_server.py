#! /usr/bin/env python3
import rospy
import actionlib
import time
import math

from math import degrees
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
from turtlebro_actions.msg import RotationAction, RotationActionFeedback, RotationActionGoal, RotationActionResult
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry


class RotateServer(object):

    _feedback = RotationActionFeedback()
    _result = RotationActionResult()
    started = False

    def __init__(self):

        rospy.init_node('rotate_server_node')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Start rotation ActionSever")

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.subscriber_odometry_cb)

        self.odom = Odometry()
        self.rate = rospy.Rate(20)
        self._as = actionlib.SimpleActionServer(
            'action_rotate', RotationAction, execute_cb=self.action_execute_cb, auto_start=False)
        self._as.start()

    def subscriber_odometry_cb(self, msg):
        self.odom = msg

    def action_execute_cb(self, goal):

        self.goal = goal
        self.start_action()

        while self.started:
            self.action_loop()
            self.rate.sleep()

    def start_action(self):

        self.started = True
        self.degree_delta = 0
        self.prev_pose = self.odom

        rospy.loginfo("Start Rotate Action with %i" % self.goal.goal)
        cmd = Twist()

        if(self.goal.goal > 0):
            cmd.angular.z = -self.goal.speed
        else:
            cmd.angular.z = self.goal.speed

        self.cmd_vel.publish(cmd)

    def action_loop(self):

        self.degree_delta += self.get_degree_diff(
            self.prev_pose.pose.pose.orientation, self.odom.pose.pose.orientation)
        self.prev_pose = self.odom

        self._result.result = self.degree_delta

        if (self.is_goal_reached(self.degree_delta, self.goal.goal)):
            self.cmd_vel.publish(Twist())
            self._as.set_succeeded(self._result)
            rospy.loginfo("Succeeded")
            self.started = False
        else:
            self._feedback.feedback = self.degree_delta
            self._as.publish_feedback(self._feedback)

        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self.cmd_vel.publish(Twist())
            self._as.set_preempted(self._result)
            self.started = False

        rospy.loginfo("Have degree %s" % self.degree_delta)

    def is_goal_reached(self, degrees, goal_val):

        if(goal_val >= 0):
            if(degrees >= goal_val):
                return True
            else:
                return False

        if(goal_val < 0):
            if(degrees < goal_val):
                return True
            else:
                return False

    def get_degree_diff(self, prev_orientation, current_orientation):

        prev_q = [prev_orientation.x, prev_orientation.y,
                  prev_orientation.z, prev_orientation.w]
        current_q = [current_orientation.x, current_orientation.y,
                     current_orientation.z, current_orientation.w]

        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)

        return math.degrees(yaw)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        if(self._as.is_active()):
            self.cmd_vel.publish(Twist())
            self._as.set_aborted(self._result)
            self.started = False
            rospy.sleep(1)


if __name__ == '__main__':
    server = RotateServer()
    rospy.spin()
