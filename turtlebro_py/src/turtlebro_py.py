import rospy
import turtlebro_actions
from math import degrees
import cv2
import numpy as np

from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

odom = Odometry()
scan = LaserScan()

len_of_scan_ranges = 1160

def subscriber_odometry_cb(msg):
    global odom
    odom = msg

def subscriber_scan_cb(msg):
    global scan
    scan = msg


colorpub = rospy.Publisher("/color_led", Int16, queue_size=10)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber("/odom", Odometry, subscriber_odometry_cb)
rospy.Subscriber("/scan", LaserScan, subscriber_scan_cb)

rospy.init_node("tb_py")

def sleep(time):
    rospy.sleep(time)

def move(meters, speed_val = 0.1):
    init_position = odom
    vel = Twist()
    fixed_inkiln = 0.01 #fixed distance (in m.) there acceleration/decceleration is performing 
    while not rospy.is_shutdown():
        if (odom.pose.pose.position.x - init_position.pose.pose.position.x < meters):
            vel.linear.x = vel_move_value(speed_val, fixed_inkiln, init_position.pose.pose.position.x, odom.pose.pose.position.x, meters) 
            print(vel.linear.x)
            vel_pub.publish(vel)
        else:
            vel.linear.x = 0
            vel_pub.publish(vel)
            return
        rospy.sleep(0.05)

def turn(degrees, speed_val = 0.5):
    degree_delta = 0
    prev_pose = odom
    init_position = 0
    vel = Twist()
    fixed_inkiln = 3 #fixed angle (in deg.) there acceleration/decceleration is performing
    while not rospy.is_shutdown():
        print(degree_delta)
        if (degree_delta < degrees):
            vel.angular.z = vel_move_value(speed_val, fixed_inkiln, init_position, degree_delta, degrees)
            vel_pub.publish(vel)
            #print(vel.angular.z)
            degree_delta += get_degree_diff(prev_pose.pose.pose.orientation, odom.pose.pose.orientation)
            prev_pose = odom
        else:
            vel.angular.z = 0
            vel_pub.publish(vel)
            return
        rospy.sleep(0.05)

def color(col):
    rgb = {"red":1, "green":2, "blue":3, "white":4, "off":5}
    shade = Int16()
    shade.data = rgb[col]
    colorpub.publish(col)

def distance(angle = 0):
    if (angle == 0):
        return scan.ranges[0]
    elif angle:
        return scan.ranges[int(angle*step_of_angles)]
    else:
        j = 0
        for i in range(len(scan.ranges)):
            k = int(j / step_of_angles)
            j += 1
            retscan[k] = i
        return retscan
    
def photo(name = "robophoto"):
    image_msg = rospy.wait_for_message("/front_camera/image_raw/compressed", CompressedImage)
    np_arr = np.frombuffer(image_msg.data, np.uint8)
    image_from_ros_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite("/home/pi/"+ name +".jpg", image_from_ros_camera)

def vel_move_value(speed, fixed_inklin, init_x, curent_x, aim_x):
    if (curent_x == init_x):
        return 0.01
    elif (curent_x < init_x):
        return (init_x - curent_x) * speed
    elif (curent_x > aim_x):
        return (aim_x - curent_x) * speed
    if ((curent_x - init_x) < fixed_inklin):
        return ((curent_x - init_x) / fixed_inklin) * speed
    elif((aim_x - curent_x) < fixed_inklin):
        return (aim_x - curent_x) * speed
    else:
        return speed
        
def get_degree_diff(prev_orientation, current_orientation):
    prev_q = [prev_orientation.x, prev_orientation.y,
                prev_orientation.z, prev_orientation.w]
    current_q = [current_orientation.x, current_orientation.y,
                    current_orientation.z, current_orientation.w]

    delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
    (_, _, yaw) = euler_from_quaternion(delta_q)
    return -degrees(yaw)

def init():
    global len_of_scan_ranges, retscan, step_of_angles
    len_of_scan_ranges = len(scan.ranges)
    step_of_angles = len_of_scan_ranges / 360
    retscan = [0 for i in range(360)]
    rospy.sleep(0.5)

init()