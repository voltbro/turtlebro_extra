import rospy
import subprocess
from math import sqrt, radians 
from math import degrees as dg
import cv2
import numpy as np

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebro_speech.srv import Speech, SpeechResponse, SpeechRequest 


DEBUG = 0

odom = Odometry()
scan = LaserScan()
names_of_func_to_call = {}

len_of_scan_ranges = 1160

def subscriber_odometry_cb(msg):
    global odom
    odom = msg

def subscriber_scan_cb(msg):
    global scan
    scan = msg

def subscriber_buttons_cb(msg):
    try:
        if(msg.data):
            names_of_func_to_call[msg.data]()
            rospy.sleep(0.5) #workaround for non lib functions
    except BaseException:
        pass

def call(name, button = 24):
    global names_of_func_to_call
    names_of_func_to_call[button] = name


colorpub = rospy.Publisher("/py_leds", Int16, queue_size=10)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber("/odom", Odometry, subscriber_odometry_cb)
rospy.Subscriber("/scan", LaserScan, subscriber_scan_cb)
rospy.Subscriber("/buttons", Int16, subscriber_buttons_cb, queue_size=1)

rospy.init_node("tb_py")

def wait(time):
    rospy.sleep(time)

def move(meters, speed_val = 0.05):
    init_position = odom
    init_x = 0
    distance_passed = 0
    epsilon = 0.005
    vel = Twist() 
    while not rospy.is_shutdown():
        distance_passed = sqrt((odom.pose.pose.position.x - init_position.pose.pose.position.x)**2 + (odom.pose.pose.position.y - init_position.pose.pose.position.y)**2)
        if (distance_passed + epsilon < meters):
            vel.linear.x = vel_x_move_value(speed_val, init_x, distance_passed, meters)
            vel_pub.publish(vel)
        else:
            vel.linear.x = 0
            vel_pub.publish(vel)
            if DEBUG:
                print("Proehal m.:", distance_passed)
            return
        rospy.sleep(0.05)

def turn(degrees, speed_val = 0.2):
    angle_delta = 0
    prev_pose = odom
    init_angle = 0
    vel = Twist()
    epsilon = 0.01
    angle = radians(degrees)
    while not rospy.is_shutdown():
        if (angle_delta + epsilon < angle):
            vel.angular.z = vel_z_turn_value(speed_val, init_angle, angle_delta, angle)
            vel_pub.publish(vel)
            angle_delta += get_angle_diff(prev_pose.pose.pose.orientation, odom.pose.pose.orientation)
            prev_pose = odom
        else:
            vel.angular.z = 0
            vel_pub.publish(vel)
            if DEBUG:
                print("Povernul gradusov:", dg(angle_delta))
            return
        rospy.sleep(0.05)

def color(col):
    rgb = {"red":1, "green":2, "blue":3, "yellow":4, "white":5, "off":6}
    shade = Int16()
    shade.data = int(rgb[col])
    colorpub.publish(shade)

def distance(angle = 0):
    if (angle == 0):
        return scan.ranges[0]
    elif angle < 360:
        return scan.ranges[int(angle*step_of_angles)]
    elif angle == 360:
        j = 0
        for i in range(len(scan.ranges)):
            k = int(j / step_of_angles)
            j += 1
            retscan[k] = i
        return retscan
    else:
        return None
    
def photo(name = "robophoto"):
    image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
    np_arr = np.frombuffer(image_msg.data, np.uint8)
    image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)
    if DEBUG:
        print("Photo zapisano v /home/pi/" + name +".jpg")

def record(timeval = 3, filename = "turtlebro_sound"):
    p = subprocess.Popen(["arecord", "-D", "hw:1,0", "-f", "S16_LE", "-r 48000", "/home/pi/" + filename + ".ogg"]) 
    rospy.sleep(timeval)
    p.kill()

def say(text = "Привет"):
    speech_service.wait_for_service()
    speech_service.call(SpeechRequest(data = text))
    

def vel_x_move_value(speed, init_x, curent_x, aim_x):
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
    
def vel_z_turn_value(speed, init_x, curent_x, aim_x):
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

def get_angle_diff(prev_orientation, current_orientation):
    prev_q = [prev_orientation.x, prev_orientation.y,
                prev_orientation.z, prev_orientation.w]
    current_q = [current_orientation.x, current_orientation.y,
                    current_orientation.z, current_orientation.w]

    delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
    (_, _, yaw) = euler_from_quaternion(delta_q)
    return -yaw

def init():
    global len_of_scan_ranges, retscan, step_of_angles, speech_service
    len_of_scan_ranges = len(scan.ranges)
    step_of_angles = len_of_scan_ranges / 360
    retscan = [0 for i in range(360)]
    speech_service = rospy.ServiceProxy('festival_speech', Speech)
    rospy.sleep(0.5)

init()