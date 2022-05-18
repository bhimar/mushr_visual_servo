#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


class DetectorNode:
    def __init__(self):
        rospy.init_node("image_detector")
        self.out_dir = rospy.get_param("~out_dir")
        self.write_images = rospy.get_param("~write_images", "false")
        self.im_topic = rospy.get_param("~im_topic", "/spoof_car/images")
        self.box_topic = rospy.get_param("~box_topic", "/spoof_car/box")

        self.bridge = CvBridge()
        
        self.sub_im = rospy.Subscriber(self.im_topic, Image, self.detect_ball)
        self.pub_img = rospy.Publisher(self.box_topic, String, queue_size=1)
       
        self.filecount = 0

        if self.write_images:
            if not os.path.exists(self.out_dir):
                os.mkdir(self.out_dir)

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
        
        mask = np.uint8(mask)
        retval, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        
        # find largest component
        area = stats[:, cv2.CC_STAT_AREA]
        area[0] = 0
        max_ind = np.argmax(area)

        # get bounding box info: centroid and dims
        cent_x = centroids[max_ind, 0]
        cent_y = centroids[max_ind, 1]
        w = stats[max_ind, cv2.CC_STAT_WIDTH]
        h = stats[max_ind, cv2.CC_STAT_HEIGHT]
        bounding_box = [cent_x, cent_y, w, h]
        
        self.pub_img.publish(str(bounding_box))

        # write bounding box images for dev and debug
        if self.write_images:
            x = stats[max_ind, cv2.CC_STAT_LEFT]
            y = stats[max_ind, cv2.CC_STAT_TOP]
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 4)
            cv2.imwrite(os.path.join(self.out_dir, str(self.filecount) + ".jpg"), frame)
            self.filecount += 1
            print("wrote file: " + str(self.filecount))


if __name__ == "__main__":
    DetectorNode()
