#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os

class PathParserNode:
    def __init__(self):
        rospy.init_node("path_parser")
        self.box_topic = rospy.get_param("~box_topic", "/spoof_car/box")
        self.move_topic = rospy.get_param("~move_topic", "/spoof_car/vel_angle")
        
        self.sub_box = rospy.Subscriber(self.box_topic, String, self.get_vel_angle)
        self.pub_move = rospy.Publisher(self.move_topic, String, queue_size=1)
        
        rospy.sleep(1.0)
        rospy.spin()
        
    def get_vel_angle(self, bounding_box):
        self.pub_move.publish("0.1, 0.2")


if __name__ == "__main__":
    PathParserNode()




    
