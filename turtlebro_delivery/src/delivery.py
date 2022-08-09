import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16

from turtlebro_aruco.srv import ArucoDetect, ArucoDetectResponse, ArucoDetectRequest

class TopCap():
    state = 'unknow'
    end_switch = 0

    def __init__(self) -> None:

        self.end_switch_sub = rospy.Subscriber("/end_switch", Int16, self.end_switch_cb)
        self.top_cap_pub    = rospy.Publisher("/top_cap", Int16, queue_size=5)
        
        rospy.sleep(1)
        rospy.loginfo("TopCap init done")

        self.open()


    def open(self):
        if self.state != 'open':
            rospy.loginfo('Open top cap')
            self.state = 'open'
            self.top_cap_pub.publish(1)

    def close(self):
        if self.state == 'open':
            rospy.loginfo('Close top cap')
            self.state = 'closed'
            self.top_cap_pub.publish(0)

    def is_closed(self):
        return bool(self.end_switch)    

    def end_switch_cb(self, msg):
        self.end_switch = msg.data

class Button():

    button = 0

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/top_button", Int16, self.button_cb)
        rospy.loginfo("TopButton init done")

    def is_pressed(self):            
        return bool(self.button)

    def button_cb(self, msg):
        self.button = msg.data        

class DeliveryRobot():
    def __init__(self) -> None:
        rospy.on_shutdown(self.on_shutdown)
        self.rate = rospy.Rate(10)
        self.product_id = 0
        self.state = 'product_wait'
        self.top_cap = TopCap()
        self.start_button = Button()

        self.aruco_service = rospy.ServiceProxy('aruco_detect', ArucoDetect)
        rospy.loginfo(f"Waiting for aruco service")
        self.aruco_service.wait_for_service()
        rospy.loginfo(f"Have aruco service")

        rospy.sleep(1)
        rospy.loginfo("Init done")

    def spin(self):

        while not rospy.is_shutdown():

            if self.state in ['product_wait','start_button_wait']:
                self.top_cap.open()
                aruco_result = self.aruco_service.call(ArucoDetectRequest())
                if aruco_result.id > 0:
                    self.product_id = aruco_result.id
                    self.state = "start_button_wait"
                    rospy.loginfo(f"Have product: {self.product_id}")

            if self.state == 'start_button_wait' :
                if self.start_button.is_pressed():
                    self.state = 'cap_close_wait'
                    self.top_cap.close()

            if self.state == 'cap_close_wait' :
                if self.top_cap.is_closed():
                    self.state = 'on_movement'
                    #todo add movment action call
                    rospy.loginfo(f"On my way")

            self.rate.sleep()

    def on_shutdown(self):
        
        rospy.loginfo("Shutdown Delivery bot")
        # self.cmd_pub.publish(Twist()) 
        # self.client.action_client.stop()
        rospy.sleep(0.5)    



if __name__ == '__main__':
    try:
        rospy.init_node('delivery_node')
        robot = DeliveryRobot()
        robot.spin()

    except rospy.ROSInterruptException:

        robot.on_shutdown()
        rospy.loginfo("Delivery stopped due to ROS interrupt")
