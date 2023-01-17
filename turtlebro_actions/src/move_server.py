#! /usr/bin/env python3
import rospy
import actionlib
import time
import math


from turtlebro_actions.msg import MoveAction, MoveActionFeedback, MoveActionGoal, MoveActionResult
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry


class MoveServer(object):

    _feedback = MoveActionFeedback()
    _result = MoveActionResult()
    started = False

    def __init__(self):

        rospy.init_node('move_server_node', log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Start move ActionSever")

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.subscriber_odometry_cb)

        self.odom = Odometry()
        self.rate = rospy.Rate(20)
        self._as = actionlib.SimpleActionServer(
            'action_move', MoveAction, execute_cb=self.action_execute_cb, auto_start=False)
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
        self.start_pose = self.odom

        rospy.loginfo("Start Move Action distance: %s, speed: %s",
                      self.goal.goal, self.goal.speed)

        cmd = Twist()

        if(self.goal.goal > 0):
            cmd.linear.x = self.goal.speed
        else:
            cmd.linear.x = -self.goal.speed

        self.cmd_vel.publish(cmd)

    def action_loop(self):

        distance = self.get_distance(
            self.start_pose.pose.pose.position, self.odom.pose.pose.position)
        rospy.loginfo("Covered distance %s", distance)

        self._result.result = distance

        if (self.is_goal_reached(distance, self.goal.goal)):
            self.cmd_vel.publish(Twist())
            self._as.set_succeeded(self._result)
            rospy.loginfo("Succeeded")
            self.started = False
        else:
            self._feedback.feedback = distance
            self._as.publish_feedback(self._feedback)

        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self.cmd_vel.publish(Twist())
            self._as.set_preempted(self._result)
            self.started = False

    def is_goal_reached(self, distance, goal_val):

        if (distance > abs(goal_val)):
            return True
        else:
            return False

    def get_distance(self, start_pose, current_pose):

        return math.sqrt(math.pow(start_pose.x - current_pose.x, 2) + math.pow(start_pose.y - current_pose.y, 2))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        if(self._as.is_active()):
            self.cmd_vel.publish(Twist())
            self._as.set_aborted(self._result)
            self.started = False
            rospy.sleep(1)


if __name__ == '__main__':
    server = MoveServer()
    rospy.spin()
