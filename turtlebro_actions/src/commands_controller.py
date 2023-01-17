#! /usr/bin/env python3
import sys
import rospy
import actionlib
import struct

from move_client import MoveClient
from rotate_client import RotateClient
from video_client import VideoClient
from servo_client import ServoClient

class CommandsController(object):

    def __init__(self, linear_speed = 0.22, angular_speed = 1):

        rospy.loginfo("Init clients")

        self.linear_speed = linear_speed 
        rospy.loginfo("Set linear speed %f", self.linear_speed)

        self.angular_speed = angular_speed 
        rospy.loginfo("Set angular speed %f", self.angular_speed)

        self.movie_client = MoveClient()
        self.rotate_client = RotateClient()
        self.video_client = VideoClient()
        self.servo44 = ServoClient(44)
        self.servo45 = ServoClient(45)

    def execute(self, command, value):

        rospy.loginfo("Execute: %s, value: %s", command, value)

        #set up linear speed
        if(command == 1):
            self.linear_speed = value/100.0
            rospy.loginfo("Set linear speed %f", self.linear_speed)

        #set up linear speed
        if(command == 2):
            self.angular_speed = value/1000.0  
            rospy.loginfo("Set angular speed %f", self.angular_speed)

        #move_forward
        if(command == 11):
            distance = value/100.0
            return self.movie_client.SendGoal(distance, self.linear_speed)

        #move_backward    
        if(command == 12):
            distance = value/100.0
            return self.movie_client.SendGoal(-distance, self.linear_speed)

        #negative Z rotattion
        if(command == 15):
            return self.rotate_client.SendGoal(value, self.angular_speed)    

        #positive Z rotattion
        if(command == 16):
            return self.rotate_client.SendGoal(-value, self.angular_speed)    

        # camera X rotation
        if(command == 21):
            return self.servo44.move(value)    

        # camera X rotation
        if(command == 22):
            return self.servo45.move(value)        

        # rospy.sleep
        if(command == 41):
            rospy.sleep(value/1000)           

        # get image data
        if(command == 55):
            return self.video_client.getImage()

        # get mock data
        if(command == 56):
            file_data = ""           
            for i in range(0, value):
                file_data += "1"

            return file_data   


if __name__ == '__main__':

    rospy.init_node('commands_control_node')
    controll = CommandsController()

    struct_format = '!BH'
    struct_len = struct.calcsize(struct_format)
    # chunks = b'\x01\xfb\x18\x01\xfb\x18'
    chunks = struct.pack(struct_format, 11, 30)
    chunks += struct.pack(struct_format, 12, 30)
    chunks += struct.pack(struct_format, 15, 30)
    chunks += struct.pack(struct_format, 16, 30)
    # chunks = struct.pack(struct_format, 55, 00)


    commands_count  = int(len(chunks)/struct_len)

    for i in range(0, commands_count):
        try:
            (command, value) = struct.unpack(struct_format,chunks[i*struct_len:(i+1)*struct_len])
            print(command, value) 
            controll.execute(command, value)
        except:
            pass    
