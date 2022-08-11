#!/usr/bin/env python3

import rospy, math

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse, ArucoDetectRequest

from config import config_raw, speech_params
from devices import Button, TopCap
from speech_client import SpeechClient
from tf.transformations import quaternion_from_euler
   

class DeliveryRobot():
    def __init__(self, delivery_config, speech_params) -> None:
        rospy.on_shutdown(self.on_shutdown)

        self.delivery_config = delivery_config

        self.rate = rospy.Rate(5)
        self.client: dict = {}
        self.state: str = 'product_wait'

        self.top_cap = TopCap()
        self.start_button = Button()
        self.speech = SpeechClient(speech_params)

        self.aruco_service = rospy.ServiceProxy('aruco_detect', ArucoDetect)
        rospy.loginfo(f"Waiting for aruco service")
        self.aruco_service.wait_for_service()
        rospy.loginfo(f"Have aruco service")


        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting move_base action")
        self.move_base_client.wait_for_server()
        rospy.loginfo(f"Have move_base action")

        # rospy.sleep(1)
        self.speech.say('start')
        rospy.loginfo("Init done")



    def spin(self):

        while not rospy.is_shutdown():

            # точка загрузки, ждем код в камеру
            if self.state in ['product_wait']:
                self.top_cap.open()

                aruco_result: ArucoDetectResponse = self.aruco_service.call(ArucoDetectRequest())

                if aruco_result.id > 0:
                    self.client = self._find_client_by_product(aruco_result.id)

                    if self.client:
                        self.state = "start_button_wait"
                        rospy.loginfo(f"Have {self.client['name']} product: {aruco_result.id}")
                        self.speech.say('have_product')
                    else:
                        rospy.loginfo(f"No client for product {aruco_result.id}")  
                        self.speech.say('have_product_error')

            # точка загрузки, ждем нажатие кнопки
            if self.state == 'start_button_wait' :
                if self.start_button.is_pressed():
                    self.state = 'home_cap_close_wait'
                    self.top_cap.close()

            # точка загрузки, ждем закрытие крышки и едем
            if self.state == 'home_cap_close_wait' :
                if self.top_cap.is_closed():
                    self.state = 'move_to_delivery_point'

                    self.speech.say('move_to_delivery_point')
                    rospy.loginfo(f"On {self.client['name']} way")

                    goal = self._goal_message_assemble(self.client['pose'])
                    self.move_base_client.send_goal(goal, done_cb=self.move_client_cb)

            # точка клиента, ждем аруко код
            if self.state == 'on_delivery_point' :   
                aruco_result: ArucoDetectResponse = self.aruco_service.call(ArucoDetectRequest())
                if aruco_result.id > 0:
                    # check client secret
                    if aruco_result.id == self.client['secret']:
                        rospy.loginfo(f"Client secret confirmed")   
                        self.state = "client_pickup_wait"  
                        self.top_cap.open()   
                        self.client = {}
                        self.speech.say('client_pickup_wait') 
                    else :
                        rospy.loginfo(f"Wrong client secret")      

            # точка клиента, ждем когда заберут и нажмут кнопку
            if self.state == 'client_pickup_wait' : 
                if self.start_button.is_pressed():
                    self.state = 'client_cap_close_wait'
                    self.top_cap.close()

            # точка клиента, ждем когда крышка будет закрыта и едем домой
            if self.state == 'client_cap_close_wait' :        
                    self.top_cap.close()
                    self.state = 'move_to_home_point'
                    self.speech.say("move_to_home_point")
                    rospy.loginfo(f"On home way")

                    goal = self._goal_message_assemble(self.delivery_config['home']['pose'])
                    self.move_base_client.send_goal(goal, done_cb=self.move_home_cb)

            # приехали домой, переходим в ожидание загрузки
            if self.state == 'on_home_point':
                self.top_cap.open()
                self.state = 'product_wait'
                self.speech.say('start')
                rospy.loginfo(f"Waiting for new delivery")
           
            self.rate.sleep()

    def move_client_cb(self, status, result):

        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Delivery point cancelled")
            self.state = "product_wait"
            self.speech.say('delivery_error')   
            

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Delivery point reached")
            self.speech.say('on_delivery_point')            
            self.state = "on_delivery_point"

    def move_home_cb(self, status, result):

        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Move to home point cancelled")
            self.state = "product_wait"
            self.speech.say('delivery_error')   

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Move to home point reached")            
            self.state = "on_home_point"


    def _find_client_by_product(self, product) -> dict():
        for key,value in self.delivery_config.items():
            if key != 'home':
                if product in value['products']: 
                    value['name'] = key
                    return value

        return dict()    

    def _goal_message_assemble(self, pose):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(pose['x'])
        goal.target_pose.pose.position.y = float(pose['y'])

        q = quaternion_from_euler(0, 0, math.radians(float(pose['theta']))) 
    
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Created goal from point {} ".format(pose))

        return goal                

    def on_shutdown(self):
        
        rospy.loginfo("Shutdown Delivery bot")
        self.move_base_client.action_client.stop()
        rospy.sleep(0.5)    



if __name__ == '__main__':
    try:
        rospy.init_node('delivery_node')

        robot = DeliveryRobot(
                    delivery_config = config_raw,
                    speech_params = speech_params)

        robot.spin()

    except rospy.ROSInterruptException:

        robot.on_shutdown()
        rospy.loginfo("Delivery stopped due to ROS interrupt")
