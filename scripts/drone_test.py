#!/usr/bin/env python2

import rospy
from drone import Drone
import numpy as np


rospy.init_node('drone_control', anonymous=True)
drone = Drone()

drone.arm()
drone.takeoff(1.0)
drone.land()
