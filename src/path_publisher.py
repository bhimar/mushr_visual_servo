#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import os 

class PathPublisher:
	def __init__(self):
	    rospy.init_node("path_publisher")
	    self.control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
	    self.init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
	    self.move_topic = rospy.get_param("~move_topic", "/spoof_car/vel_angle")

   
	    self.pub_controls = rospy.Publisher(self.control_topic, AckermannDriveStamped, queue_size=1)
	    self.pub_init_pose = rospy.Publisher(self.init_pose_topic, PoseWithCovarianceStamped, queue_size=1)
	    
	    init = "0, 0, 0"
	    self.send_init_pose(init)

	    self.command = rospy.Subscriber(self.move_topic, String, self.send_command)

	#    plan_file = rospy.get_param("~plan_file")

	#    with open(plan_file) as f:
	#        plan = f.readlines()

    

	    # Publishers sometimes need a warm-up time, you can also wait until there
	    # are subscribers to start publishing see publisher documentation.
	    rospy.sleep(1.0)
	    rospy.spin()
	    # run_plan(pub_init_pose, pub_controls, plan)


	def send_init_pose(self, init_pose):
	    pose_data = init_pose.split(",")
	    assert len(pose_data) == 3

	    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
	    q = Quaternion(*quaternion_from_euler(0, 0, theta))
	    point = Point(x=x, y=y)
	    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
	    self.pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))


	def send_command(self, c):
	    
	    cmd = c.data.split(",")
	    assert len(cmd) == 2
	    v, delta = float(cmd[0]), float(cmd[1])
	    dur = rospy.Duration(1.0)
	    rate = rospy.Rate(10)
	    start = rospy.Time.now()

	    drive = AckermannDrive(steering_angle=delta, speed=v)

	    while rospy.Time.now() - start < dur:
		self.pub_controls.publish(AckermannDriveStamped(drive=drive))
		rate.sleep()


if __name__ == "__main__":
    PathPublisher()
