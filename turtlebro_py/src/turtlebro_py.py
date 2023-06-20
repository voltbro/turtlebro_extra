import rospy
import actionlib
import subprocess
from math import sqrt, radians, atan2 
from math import degrees as dg
import cv2
import numpy as np

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


DEBUG = 1

class TurtleBro():

    def __init__(self):
        rospy.init_node("tb_py")
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = Odometry()

        self.u = Utility()
             
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9
     
        rospy.sleep(0.5)

    def forward(self, meters):
        self.__move(meters)

    def backward(self, meters):
        self.__move(-meters)

    def right(self, degrees):
        self.__turn(-degrees) 

    def left(self, degrees):
        self.__turn(degrees)

    def goto(self, x, y):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        self.__turn(heading)
        self.__move(distance)

    def call(self, name, button = 24):
        self.u.__call(name, button)
    
    def wait(self, time):
        self.u.__wait(time)

    def color(self, col):
        self.u.__color(col)

    def speed(self, value):
        Kp = 10
        speed_dict = {"fastest":0.17, "fast":0.12, "normal":0.09, "slow":0.04, "slowest":0.01}
        if type(value) == str:
            self.linear_x_val = speed_dict[value]
        else:
            self.linear_x_val = Utility.__clamp(0.01, value, 0.17)
            self.angular_z_val = Kp * self.linear_x_val

    def __subscriber_odometry_cb(self, msg):
        self.odom = msg

    def __move(self, meters):
        if DEBUG:
            print("init x: ", self.odom.pose.pose.position.x, "y: ", self.odom.pose.pose.position.y)
            print("meters: ", meters)
        init_position = self.odom
        init_x = 0
        distance_passed = 0
        epsilon = 0.005
        vel = Twist() 
        while not rospy.is_shutdown():
            distance_passed = sqrt((self.odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
            if (distance_passed + epsilon < abs(meters)):
                if meters > 0:
                    vel.linear.x = self.__vel_x_move_value(self.linear_x_val, init_x, distance_passed, meters)
                else:
                    vel.linear.x = - self.__vel_x_move_value(self.linear_x_val, init_x, distance_passed, abs(meters))
                self.vel_pub.publish(vel)
            else:
                vel.linear.x = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print("Proehal m.:", distance_passed)
                return
            rospy.sleep(0.05)

    def __turn(self, degrees):
        angle_delta = 0
        prev_pose = self.odom
        init_angle = 0
        vel = Twist()
        epsilon = 0.03
        angle = radians(degrees)
        while not rospy.is_shutdown():
            if (abs(angle_delta) + epsilon < abs(angle)):
                if angle > 0:
                    vel.angular.z = self.__vel_z_turn_value(self.angular_z_val, init_angle, angle_delta, angle)
                else:
                    vel.angular.z = -self.__vel_z_turn_value(self.angular_z_val, init_angle, abs(angle_delta), abs(angle))
                angle_delta += self.__get_angle_diff(prev_pose.pose.pose.orientation, self.odom.pose.pose.orientation)
                self.vel_pub.publish(vel)
                prev_pose = self.odom
            else:
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                if DEBUG:
                    print("Povernul gradusov:", dg(angle_delta))
                return
            rospy.sleep(0.05)
    
    def __vel_x_move_value(self, speed, init_x, curent_x, aim_x):
        fixed_inklin = 0.01 #fixed distance (in m.) there acceleration/decceleration is performing
        if (curent_x == init_x):
            return 0.01
        elif (curent_x < init_x):
            return (init_x - curent_x) * speed
        elif (curent_x > aim_x):
            return 0
        elif ((curent_x - init_x) < fixed_inklin):
            return ((curent_x - init_x) / fixed_inklin) * speed
        elif((aim_x - curent_x) < fixed_inklin):
            return (aim_x - curent_x) * speed
        else:
            return speed
        
    def __vel_z_turn_value(self, speed, init_x, curent_x, aim_x):
        fixed_inklin = 0.08 #fixed angle (in deg.) there acceleration/decceleration is performing
        Kp = 2
        if (curent_x == init_x):
            return 0.01
        elif (curent_x < init_x):
            return (init_x - curent_x) * speed * Kp
        elif (curent_x > aim_x):
            return 0
        elif ((curent_x - init_x) < fixed_inklin):
            return ((curent_x - init_x) / fixed_inklin) * speed * Kp
        elif((aim_x - curent_x) < fixed_inklin):
            return (aim_x - curent_x) * speed * Kp
        else:
            return speed

    def __get_angle_diff(self, prev_orientation, current_orientation):
        prev_q = [prev_orientation.x, prev_orientation.y,
                    prev_orientation.z, prev_orientation.w]
        current_q = [current_orientation.x, current_orientation.y,
                        current_orientation.z, current_orientation.w]

        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
        (_, _, yaw) = euler_from_quaternion(delta_q)
        return -yaw

    def __get_turn_angle_to_point(self, x, y):
        (_, _, yaw) = euler_from_quaternion(self.odom.pose.pose.orientation)
        heading = atan2(y,x)
        angle_to_turn = yaw - heading
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        distance = sqrt((self.odom.pose.pose.position.x - x)**2 + (self.odom.pose.pose.position.y - y)**2)
        return distance

class TurtleNav():
    
    def __init__(self):
        self.u = Utility()
        rospy.init_node("tb_nav_py")
        self.odom = Odometry()
        self.scan = LaserScan()
        
        self.names_of_func_to_call = {}

        self.linear_x_val = 0.09
        self.angular_z_val = 0.9

        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def forward(self, meters, speed_val = 0.05):
        self.__move(meters, speed_val = 0.05)

    def backward(self, meters, speed_val = 0.05):
        self.__move(-meters, speed_val = 0.05)

    def right(self, degrees, speed_val = 0.2):
        self.__turn(-degrees, speed_val = 0.2) 

    def left(self, degrees, speed_val = 0.2):
        self.__turn(degrees, speed_val = 0.2)

    def goto(self, x, y):
        self.__goto(x,y)

    def call(self, name, button = 24):
        self.u.__call(name, button)
    
    def wait(self, time):
        self.u.__wait(time)

    def color(self, col):
        self.u.__color(col)

    """
    def speed(self, value): #TODO
        Kp = 10
        speed_dict = {"fastest":0.17, "fast":0.12, "normal":0.09, "slow":0.04, "slowest":0.01}
        if type(value) == str:
            self.linear_x_val = speed_dict[value]
        else:
            self.linear_x_val = Utility.__clamp(0.01, value, 0.17)
            self.angular_z_val = Kp * self.linear_x_val
    """

    def __subscriber_odometry_cb(self, msg):
        self.odom = msg
    
    def __goal_message_assemble(self, x ,y, theta = 0):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, radians(float(theta)))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        return goal

    def __move(self, meters):
        goal = self.__goal_message_assemble(meters)
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)

    def __turn(self, degrees):
        goal = self.__goal_message_assemble(0, theta = radians(degrees))
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)


class Utility():

    def __init__(self):
        self.scan = LaserScan()
        rospy.Subscriber("/scan", LaserScan, self.__subscriber_scan_cb)
        rospy.Subscriber("/buttons", Int16, self.__subscriber_buttons_cb, queue_size=1)
        self.colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)
        
        rospy.sleep(0.5)

        odom_reset = rospy.ServiceProxy('reset', Empty)
        odom_reset.wait_for_service()
        #odom_reset.call(Empty())

        self.len_of_scan_ranges = len(self.scan.ranges)
        self.step_of_angles = self.len_of_scan_ranges / 360
        self.retscan = [0] * 360
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
        

    def __call(self, name, button = 24):
        self.names_of_func_to_call[button] = name
    
    def __wait(self, time):
        rospy.sleep(time)

    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg):
        try:
            if(msg.data):
                self.names_of_func_to_call[msg.data]()
                rospy.sleep(0.5) #workaround for non lib functions
        except BaseException:
            pass
    
    def __color(self, col):
        rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    def __distance(self, angle = 0):
        if (angle == 0):
            return self.scan.ranges[0]
        elif angle < 360:
            return self.scan.ranges[int(angle * self.step_of_angles)]
        elif angle == 360:
            j = 0
            for i in range(self.len_of_scan_ranges):
                k = int(j / self.step_of_angles)
                j += 1
                self.retscan[k] = i
            return self.retscan
        else:
            return None
    
    def __photo(self, name = "robophoto"):
        image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)
        if DEBUG:
            print("Photo zapisano v /home/pi/" + name +".jpg")

    def __record(self, timeval = 3, filename = "turtlebro_sound"):
        p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + ".ogg"]) 
        rospy.sleep(timeval)
        p.kill()

    def __say(self, text = "Привет"):
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))

    def __clamp(min_val, value, max_val):
        return max(min_val, min(value, max_val))