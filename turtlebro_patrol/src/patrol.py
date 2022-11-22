#!/usr/bin/env python3
import rospy
import math
import toml

from itertools import cycle

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
from turtlebro_patrol.srv import PatrolControlCallback, PatrolControlCallbackRequest

#Import standard Pose msg types and TF transformation to deal with quaternions
from tf.transformations import quaternion_from_euler

from pathlib import Path



class Patrol(object):

    def __init__(self):

        rospy.on_shutdown(self.on_shutdown)

        self.goal = None 
        self.patrol_state = "patrol"
        self.load_config_file()   
        self.current_point = self.config['home'] 


        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.reached_point_pub = rospy.Publisher('/patrol_control/reached', PatrolPoint, queue_size=5)
        rospy.Service('patrol_control', PatrolControlCallback, self.control_service)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting move_base action")
        self.client.wait_for_server()

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
 
    def control_service(self, message):
          
        if message.command in ["start", "pause", "resume", "home", "shutdown"]:

            rospy.loginfo("Patrol: {} sequence received".format(message.command))

            self.client.cancel_all_goals()  

            if message.command == "shutdown":
                result = "Ok, goodbye"
                self.patrol_state = "shutdown"  
                
            if message.command in ["start", "resume"]:    
                result = "Ok, let`s do it"
                self.patrol_state = "patrol"

            if message.command in ["home", "pause"]:    
                self.patrol_state = "wait"
                result = "Ok, let`s do it"

            # start / resume movement opp 
            if message.command in ["resume", "home", "start"]:
                patrol_point = self.get_patrol_point(message.command)
                self.goal = self.goal_message_assemble(patrol_point)    
        else:
            rospy.loginfo("Patrol: Command unrecognized")
        
        return(result)

    def move(self):
        
        # in that loop we will check if there is shutdown flag or rospy core have been crushed
        while not rospy.is_shutdown():
            if self.patrol_state == "shutdown":
                rospy.sleep(0.1)
                rospy.signal_shutdown("Have shutdown command in patrol_control service")
            else:
                if self.goal is not None:
                    self.client.send_goal(self.goal, done_cb=self.move_base_cb)
                    self.goal = None

            rospy.sleep(0.1)


    def get_patrol_point(self, command):
        # command: [start resume next home]

        if command in ['resume', 'current']:
            pass

        if command == 'home':
            self.current_point = self.config['home']

        if command == 'start':
            self.patrolling = cycle(self.config['patrolling'])
            self.current_point = next(self.patrolling)

        if command == 'next':  # cycle patrol points 
            self.current_point = next(self.patrolling)
            
        return self.current_point

    def move_base_cb(self, status, result):

        point = self.get_patrol_point('current')

        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Patrol: Goal cancelled {}".format(point['name']))

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Patrol: Goal reached {}".format(point['name']))

            patrol_point = PatrolPoint(
                x = float(point['pose']['x']),
                y = float(point['pose']['y']),
                theta = int(point['pose']['theta']),
                name = point['name'])

            self.reached_point_pub.publish(patrol_point)        

            if self.call_back_service:
                rospy.loginfo("Call patrol Service")
                request = PatrolPointCallbackRequest()
                request.patrol_point = patrol_point
                self.call_back_service.call(request)
                rospy.loginfo("Call patrol Service: finish")

            # renew patrol point if on patrol mode
            if self.patrol_state == "patrol":
                next_point = self.get_patrol_point('next')
                rospy.sleep(0.5)  # small pause in point
                self.goal = self.goal_message_assemble(next_point)  

    def load_config_file(self):

        rospy.loginfo("Patrol: TOML Parsing started")

        try:
        
            config_data_file = rospy.get_param('~config_data_file', str(
            Path(__file__).parent.absolute()) + '/../data/goals.toml')

            rospy.loginfo(f"Loading config file {config_data_file}")

            self.config = toml.load(config_data_file)  
            self.config['home']['name'] = 'home'
            rospy.loginfo(f"Patrol Points: {self.config }")

            # TODO check home point exist, check patrolling data exist

        except Exception as e:

            rospy.loginfo("TOML parser failed")
            rospy.signal_shutdown("No valid TOML file")


    def goal_message_assemble(self, point):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point['pose']['x'])
        goal.target_pose.pose.position.y = float(point['pose']['y'])

        q = quaternion_from_euler(0, 0, math.radians(float(point['pose']['theta']))) # using TF.transformation func to get quaternion from theta Euler angle
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Patrol: created goal from point {} ".format(point['name']))

        return goal
        
    def on_shutdown(self):
        
        rospy.loginfo("Shutdown my patrol")
        self.cmd_pub.publish(Twist()) 
        self.client.action_client.stop()
        rospy.sleep(0.5)     

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('turtlebro_patrol')
        patrol = Patrol()
        patrol.move()

    except rospy.ROSInterruptException:

        patrol.on_shutdown()
        rospy.loginfo("Patrol stopped due to ROS interrupt")
