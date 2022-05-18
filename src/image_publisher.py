#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import os

def publish_images(pub_im, data_dir):
    bridge = CvBridge()

    for filename in os.listdir(data_dir):
        cv_image = cv2.imread(os.path.join(data_dir, filename))
        if cv_image is not None:
            img_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            pub_im.publish(img_msg)
        rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('image_publisher')

    data_dir = rospy.get_param("~data_dir")
    im_topic = rospy.get_param("~im_topic","/spoof_car/images")
    pub_im = rospy.Publisher(im_topic, Image, queue_size=1)
    
    rospy.sleep(5.0)
    publish_images(pub_im, data_dir)

    
