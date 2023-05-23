import rospy
import actionlib
import turtlebro_actions
import math

from std_msgs.msg import Int16
from turtlebro_actions.msg import MoveAction, MoveGoal
from turtlebro_actions.msg import RotationAction, RotationGoal

moveclient = actionlib.SimpleActionClient('action_move', MoveAction)
rclient = actionlib.SimpleActionClient('action_rotate', RotationAction)
colorpub = rospy.Publisher("/color_led", Int16, queue_size=10)

rclient.wait_for_server()

rgb = {"red":1, "green":2, "blue":3, "white":4, "off":5}

rospy.init_node("tb_py")

def sleep(time):
    print("sleeping for: ", time)
    rospy.sleep(time)

def move(meters, speed_val = 0.1):
    goal = MoveGoal(goal = meters, speed = speed_val)
    moveclient.send_goal(goal)
    moveclient.wait_for_result()
    return moveclient.get_result()

def turn(degrees, speed_val = 0.5):
    goal = MoveGoal(goal = math.radians(degrees), speed = math.radians(speed_val))
    rclient.send_goal(goal)
    rclient.wait_for_result()
    return rclient.get_result()

def color(col):
    shade = Int16()
    shade.data = rgb[col]
    colorpub.publish(col)