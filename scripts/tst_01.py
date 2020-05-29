#!/usr/bin/env python2
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time
class Drone:
    def __init__(self):
        self.pose = None
        self.q = None
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
	rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        
    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.q = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])
    def angle(self):
	rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        print(self.pose)
        (roll, pitch, yaw) = euler_from_quaternion (self.q)
        print(self.pose,(yaw*180)/np.pi)
        self.rate.sleep()

if __name__ == "__main__":
 rospy.init_node('drone', anonymous=True)
 drone = Drone()
 key=str('a')
 while key != str('s'):
   drone.angle()
   key=input()

