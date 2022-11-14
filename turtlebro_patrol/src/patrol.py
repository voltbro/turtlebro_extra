#!/usr/bin/env python3
import rospy
import math

#import Twist data type for direct control
from geometry_msgs.msg import Twist

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from turtlebro_patrol.msg import PatrolPoint
from turtlebro_patrol.srv import PatrolPointCallback, PatrolPointCallbackRequest

#Import standard Pose msg types and TF transformation to deal with quaternions
from tf.transformations import quaternion_from_euler

from pathlib import Path

# XML parser
import xml.etree.ElementTree as ET

class Patrol(object):

    def __init__(self):

        rospy.on_shutdown(self.on_shutdown)

        rospy.Subscriber('patrol_control', String, self.patrol_control_cb)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.reached_point_pub = rospy.Publisher('/patrol_control/reached', PatrolPoint, queue_size=5)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting move_base action")
        self.client.wait_for_server()

        self.on_patrol = True
        self.current_point = 0
        self.goal = None

        self.home_point = [0, 0, 0, 'home']  # position x y theta of home
        self.patrol_points = []

        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', str(
            Path(__file__).parent.absolute()) + '/../data/goals.xml')

        service_name = rospy.get_param('~point_callback_service', False)

        self.init_callback_service(service_name)

        rospy.loginfo("Init done")

    def init_callback_service(self, service_name):

        if service_name:

            self.call_back_service = rospy.ServiceProxy(service_name, PatrolPointCallback)
            rospy.loginfo(f"Waiting point callback service : {service_name}")
            self.call_back_service.wait_for_service()
            rospy.loginfo(f"Init service: {service_name}")

        else:
            self.call_back_service = False
            rospy.loginfo("No point callback service")
 

    def move(self):

        self.patrol_points = self.fetch_points(self.waypoints_data_file)

        # in that loop we will check if there is shutdown flag or rospy core have been crushed
        while not rospy.is_shutdown():
            if self.goal is not None:
                self.client.send_goal(self.goal, done_cb=self.move_base_cb)
                self.goal = None

            rospy.sleep(0.1)

    def patrol_control_cb(self, message):

        if message.data in ["start", "pause", "resume", "home", "shutdown"]:

            rospy.loginfo("Patrol: {} sequence received".format(message.data))

            self.client.cancel_all_goals()  

            if message.data == "shutdown":
                rospy.signal_shutdown("Have shutdown command in patrol_control topic")

            if message.data in ["start", "resume", "shutdown"]:    
                self.on_patrol = True
                
            if message.data in ["home", "pause"]:    
                self.on_patrol = False

            # start / resume movement opp 
            if message.data in ["resume", "home", "start"]:
                patrol_point = self.get_patrol_point(message.data)
                self.goal = self.goal_message_assemble(patrol_point)
                     
        else:
            rospy.loginfo("Patrol: Command unrecognized")


    def get_patrol_point(self, command):
        # point_type: [start current next home]

        if command == 'home':
            self.current_point = 0

        if command == 'start':
            self.current_point = 1  

        if command == 'next':
            # cycle patrol points
            self.current_point += 1
            if self.current_point >= len(self.patrol_points):
               self.current_point = 1 
            
        return self.patrol_points[self.current_point]

    def move_base_cb(self, status, result):

        point = self.patrol_points[self.current_point]

        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Patrol: Goal cancelled {}".format(point[3]))

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Patrol: Goal reached {}".format(point[3]))

            patrol_point = PatrolPoint(
                x = float(point[0]),
                y = float(point[1]),
                theta = int(point[2]),
                name = point[3])

            self.reached_point_pub.publish(patrol_point)        

            if self.call_back_service:
                rospy.loginfo("Call patroll Service")
                request = PatrolPointCallbackRequest()
                request.patrol_point = patrol_point
                self.call_back_service.call(request)
                rospy.loginfo("Call patroll Service: finish")

            # renew patrol point if on patroll mode
            if self.on_patrol:
                next_patrol_point = self.get_patrol_point('next')
                rospy.sleep(0.5)  # small pause in point
                self.goal = self.goal_message_assemble(next_patrol_point)  

    def fetch_points(self, xml_file):

        rospy.loginfo("Patrol: XML Parsing started")

        try:
            # Define xml-goals file path
            tree = ET.parse(xml_file)  # get file from launch params
            root = tree.getroot()

            points = []
            points.append(self.home_point)

            # filling points
            for xml_element in root.findall('goal'):
                points.append([xml_element.get('x'), xml_element.get('y'), xml_element.get('theta'), xml_element.get('name')])

            rospy.loginfo("Patrol:  XML parcing done. goals detected:  {}".format(points))

            return points  

        except Exception as e:

            rospy.loginfo("XML parser failed")
            rospy.signal_shutdown("No valid XML file")

    def goal_message_assemble(self, point):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point[0])
        goal.target_pose.pose.position.y = float(point[1])

        q = quaternion_from_euler(0, 0, math.radians(float(point[2]))) # using TF.transformation func to get quaternion from theta Euler angle
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Patrol: created goal from point {} ".format(point))

        return goal
        
    def on_shutdown(self):
        
        rospy.loginfo("Shutdown my patrol")
        self.cmd_pub.publish(Twist()) 
        self.client.action_client.stop()
        rospy.sleep(0.5)     

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('turtlebro_patroll')
        patrol = Patrol()
        patrol.move()

    except rospy.ROSInterruptException:

        patrol.on_shutdown()
        rospy.loginfo("Patrol stopped due to ROS interrupt")
