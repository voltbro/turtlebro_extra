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
    print("sleeping for: ", time)
    rospy.sleep(time)

def move(meters, speed_val = 0.1):
    init_position = odom
    vel = Twist()
    while not rospy.is_shutdown():
        if (odom.pose.pose.position.x - init_position.pose.pose.position.x < meters):
            vel.linear.x = vel_move_value(init_position.pose.pose.position.x, odom.pose.pose.position.x, meters)
            vel_pub.publish(vel)
        else:
            vel.linear.x = 0
            vel_pub.publish(vel)
            return
        rospy.sleep(0.2)

def turn(degrees, speed_val = 0.5):
    pass

def color(col):
    rgb = {"red":1, "green":2, "blue":3, "white":4, "off":5}
    shade = Int16()
    shade.data = rgb[col]
    colorpub.publish(col)

def distance(angle):
    if angle:
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

def vel_move_value(init_x, curent_x, aim_x):
    fixed_inklin = 0.05
    if (curent_x < init_x):
        return (init_x - curent_x)
    elif (init_x > aim_x):
        return (aim_x - init_x)
    if ((curent_x - init_x) < fixed_inklin):
        return ((curent_x - init_x) / fixed_inklin)
    elif((aim_x - curent_x) < fixed_inklin):
        return ((aim_x - curent_x) / fixed_inklin)
    else:
        return 1

def get_degree_diff(prev_orientation, current_orientation):
    prev_q = [prev_orientation.x, prev_orientation.y,
                prev_orientation.z, prev_orientation.w]
    current_q = [current_orientation.x, current_orientation.y,
                    current_orientation.z, current_orientation.w]

    delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
    (_, _, yaw) = euler_from_quaternion(delta_q)
    return degrees(yaw)

def init():
    global len_of_scan_ranges, retscan, step_of_angles
    len_of_scan_ranges = len(scan.ranges)
    step_of_angles = len_of_scan_ranges / 360
    retscan = [0 for i in range(360)]

init()