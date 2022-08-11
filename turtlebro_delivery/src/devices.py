from std_msgs.msg import Int16
import rospy

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

    def end_switch_cb(self, msg:Int16):
        self.end_switch = msg.data

class Button():

    button = 0

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/top_button", Int16, self.button_cb)
        rospy.loginfo("TopButton init done")

    def is_pressed(self):            
        return bool(self.button)

    def button_cb(self, msg:Int16):
        self.button = msg.data   
      