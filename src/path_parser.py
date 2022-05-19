#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import numpy as np

class PathParserNode:
    def __init__(self):
        rospy.init_node("path_parser")
        self.box_topic = rospy.get_param("~box_topic", "/image_detector/box")
        self.move_topic = rospy.get_param("~move_topic", "/path_parser/vel_angle")
        
        self.sub_box = rospy.Subscriber(self.box_topic, String, self.get_vel_angle)
        self.pub_move = rospy.Publisher(self.move_topic, String, queue_size=1)
        
        rospy.sleep(1.0)
        rospy.spin()
        
    def get_vel_angle(self, bounding_box):
        # self.pub_move.publish("0.1, 0.2")
        x_center, y_center, x_width, y_height = bounding_box.data[1:-1].split(', ')
        # center point should control angle
        #720 height 1280 width
        
        angle = (640 - float(x_center)) * np.pi / 2560
        
        velocity = "0.25"
        if float(y_height) > 500:
            velocity = "0"

        vel_angle = velocity + ", " + str(angle)

        # height/width of bounding box controls the velocity
        self.pub_move.publish(vel_angle)

if __name__ == "__main__":
    PathParserNode()




    
