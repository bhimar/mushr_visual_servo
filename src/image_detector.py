#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


class DetectorNode:
    def __init__(self):
        rospy.init_node("image_detector")
        self.data_dir = rospy.get_param("~data_dir")
        self.im_topic = rospy.get_param("~im_topic", "/spoof_car/images")
        self.box_topic = rospy.get_param("~box_topic", "/spoof_car/box")

        self.bridge = CvBridge()
        
        self.sub_im = rospy.Subscriber(self.im_topic, Image, self.detect_ball)
        self.pub_img = rospy.Publisher(self.box_topic, String, queue_size=1)
        
        rospy.sleep(2.0)
        rospy.spin()
        

    def detect_ball(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        
        # preprocess
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # threshold
        greenLower = (29, 86, 100)
        greenUpper = (64, 255, 255)

        # get mask and filter
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        bounding_box = [2.0,2.0,3.0,4.0]       
        self.pub_img.publish(str(bounding_box))


if __name__ == "__main__":
    DetectorNode()
