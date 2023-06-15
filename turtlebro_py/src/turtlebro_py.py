import rospy
import actionlib
import subprocess
from math import sqrt, radians, atan2 
from math import degrees as dg
import cv2
import numpy as np

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


DEBUG = 0

class Robot():

    def __init__(self):
        self.u = Utility()
        rospy.init_node("tb_py")
        self.odom = Odometry()
        self.scan = LaserScan()
        
        self.names_of_func_to_call = {}

        self.linear_x_val = 0.09
        self.angular_z_val = 0.9

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)

    def forward(self, meters, speed_val = 0.05):
        self.__move(meters, speed_val = 0.05)

    def backward(self, meters, speed_val = 0.05):
        self.__move(-meters, speed_val = 0.05)

    def right(self, degrees, speed_val = 0.2):
        self.__turn(-degrees, speed_val = 0.2) 

    def left(self, degrees, speed_val = 0.2):
        self.__turn(degrees, speed_val = 0.2)

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
        self.linear_x_val = speed_dict[value]
        self.angular_z_val = Kp * self.linear_x_val

    def __subscriber_odometry_cb(self, msg):
        self.odom = msg

    def __move(self, meters):
        init_position = self.odom
        init_x = 0
        distance_passed = 0
        epsilon = 0.005
        vel = Twist() 
        while not rospy.is_shutdown():
            distance_passed = sqrt((self.odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
            if (distance_passed + epsilon < meters):
                vel.linear.x = self.__vel_x_move_value(self.linear_x_val, init_x, distance_passed, meters)
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
        epsilon = 0.05
        angle = radians(degrees)
        while not rospy.is_shutdown():
            if (angle_delta + epsilon < angle):
                vel.angular.z = self.__vel_z_turn_value(self.angular_z_val, init_angle, angle_delta, angle)
                self.vel_pub.publish(vel)
                angle_delta += self.__get_angle_diff(prev_pose.pose.pose.orientation, self.odom.pose.pose.orientation)
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
    

class Utility():

    def __init__(self):
        rospy.sleep(0.5)

        rospy.Subscriber("/scan", LaserScan, self.__subscriber_scan_cb)
        rospy.Subscriber("/buttons", Int16, self.__subscriber_buttons_cb, queue_size=1)

        self.colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)

        self.len_of_scan_ranges = len(self.scan.ranges)
        self.step_of_angles = self.len_of_scan_ranges / 360
        self.retscan = [0 for i in range(360)]
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