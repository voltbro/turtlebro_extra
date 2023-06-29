import rospy
import actionlib
import subprocess
import math
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
    """
    Простой робот. Умеет ехать вперед - forward()
    назад - backward()
    поворачивать направо и налево - right(), left()
    ехать на определенные координаты (x,y) - goto(x,y)
    зажигать светодиоды - color()
    снимать фото - photo()
    записывать звук - record()
    измерять дистанцию - distance()
    вызывать пользовательские функции при нажатии на кнопку - call()
    произносить фразы - say()
    находиться в режиме ожидания - wait()
    """

    def __init__(self):
        rospy.init_node("tb_py")
        rospy.Subscriber("/odom", Odometry, self.__subscriber_odometry_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = Odometry()

        self.u = Utility()
             
        self.linear_x_val = 0.09
        self.angular_z_val = 0.9
     
        rospy.sleep(0.3)

    def __del__(self):
        self.vel_pub.publish(Twist())
        print("Done")

    def forward(self, meters):
        assert meters > 0, "Ошибка! Количество метров должно быть положительным"
        self.__move(meters)

    def backward(self, meters):
        assert meters > 0, "Ошибка! Количество метров должно быть положительным"
        self.__move(-meters)

    def right(self, degrees):
        assert degrees > 0, "Ошибка! Количество градусов должно быть положительным"
        self.__turn(-degrees) 

    def left(self, degrees):
        assert degrees > 0, "Ошибка! Количество градусов должно быть положительным"
        self.__turn(degrees)

    def goto(self, x, y):
        heading = self.__get_turn_angle_to_point(x, y)
        distance = self.__get_distance_to_point(x, y)
        self.__turn(heading)
        self.__move(distance)

    def call(self, name, button = 24):
        self.u.call(name, button)
    
    def wait(self, time):
        self.u.wait(time)

    def color(self, col):
        self.u.color(col)
    
    def photo(self, name = "robophoto"):
        self.u.photo(name)

    def record(self, timeval = 3, filename = "turtlebro_sound"):
        self.u.record(timeval, filename)

    def say(self, text = "Привет"):
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))
    
    def distance(self, angle = 0):
        return self.u.distance(angle)

    def speed(self, value):
        assert type(value) == str, "Скорость должно быть одним из слов: fastest, fast, normal, slow, slowest"
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
            distance_passed = math.sqrt((self.odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
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
                    print("Проехал м.:", distance_passed)
                return
            rospy.sleep(0.05)

    def __turn(self, degrees):
        angle_delta = 0
        prev_pose = self.odom
        init_angle = 0
        vel = Twist()
        epsilon = 0.03
        angle = math.radians(degrees)
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
                    print("Повернул град.:", math.degrees(angle_delta))
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
        heading = math.atan2(y,x)
        angle_to_turn = yaw - heading
        return angle_to_turn

    def __get_distance_to_point(self, x, y):
        distance = math.sqrt((self.odom.pose.pose.position.x - x)**2 + (self.odom.pose.pose.position.y - y)**2)
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
        self.u.call(name, button)
    
    def wait(self, time):
        self.u.wait(time)

    def color(self, col):
        self.u.color(col)

    def photo(self, name = "robophoto"):
        self.u.photo(name)

    def record(self, timeval = 3, filename = "turtlebro_sound"):
        self.u.record(timeval, filename)

    def say(self, text = "Привет"):
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))
    
    def distance(self, angle = 0):
        return self.u.distance(angle)

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
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, math.radians(float(theta)))
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
        goal = self.__goal_message_assemble(0, theta = math.radians(degrees))
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)

    def __goto(self, x, y):
        goal = self.__goal_message_assemble(x, y)
        self.movebase_client.wait_for_server()
        self.movebase_client.send_goal_and_wait(goal)


class Utility():

    def __init__(self):
        self.scan = LaserScan()
        self.names_of_func_to_call = {}
        rospy.Subscriber("/scan", LaserScan, self.__subscriber_scan_cb)
        rospy.Subscriber("/buttons", Int16, self.__subscriber_buttons_cb, queue_size=1)
        self.colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)
        
        rospy.sleep(0.3)

        odom_reset = rospy.ServiceProxy('reset', Empty)
        odom_reset.wait_for_service()
        #odom_reset.call(Empty())                                                                       #TODO!!!!!!!

        self.len_of_scan_ranges = len(self.scan.ranges)
        self.step_of_angles = self.len_of_scan_ranges / 360
        self.retscan = [0] * 360
        self.speech_service = rospy.ServiceProxy('festival_speech', Speech)
    
    def __del__(self):
        self.color("blue")

    def __subscriber_scan_cb(self, msg):
        self.scan = msg

    def __subscriber_buttons_cb(self, msg):
        try:
            if(msg.data):
                self.names_of_func_to_call[msg.data]()
                rospy.sleep(0.5) #workaround for non lib functions
        except BaseException:
            pass

    def call(self, name, button = 24):
        self.names_of_func_to_call[button] = name
    
    def wait(self, time = 0):
        if time == 0:
            rospy.spin()
        else:
            rospy.sleep(time)

    def color(self, col):
        assert type(col) == str, "Имя цвета должно быть строкой"
        rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
        shade = Int16()
        shade.data = int(rgb[col])
        self.colorpub.publish(shade)

    def distance(self, angle):
        assert type(angle) == (int or float), "Угол должен быть числом"
        if (angle == 0):
            return self.scan.ranges[0]
        elif angle < 360:
            return self.scan.ranges[int(angle * self.step_of_angles)]
        elif angle == 360:
            for i in range(self.len_of_scan_ranges):
                k = int(i / self.step_of_angles)
                self.retscan[k] = self.scan.ranges[k]
            return self.retscan
        else:
            return None
    
    def photo(self, name = "robophoto"): 
        assert type(name) == str, "Имя файла фото должно быть строкой"
        image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)
        if DEBUG:
            print("Фото записано в /home/pi/" + name +".jpg")

    def record(self, timeval, filename):
        assert timeval > 0 and type(timeval) == (float or int), "Временной интервал должен быть положительным числом"
        p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + ".ogg"]) 
        rospy.sleep(timeval)
        p.kill()

    def say(self, text = "Привет"):
        assert type(text) == str, "Текст должен быть строкой"
        self.speech_service.wait_for_service()
        self.speech_service.call(SpeechRequest(data = text))

    def __clamp(min_val, value, max_val):
        return max(min_val, min(value, max_val))