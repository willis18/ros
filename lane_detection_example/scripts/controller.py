#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import purePursuit,PID_longitudinal

if __name__ == "__main__":
    rospy.init_node('traffic_control', anonymous = True)

    #ctrl_lon = PID_longitudinal(K=500)

    ctrl_lat = purePursuit(lfd = 0.85)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if ctrl_lat.lpath is not None:

            ctrl_lat.steering_angle()

            ctrl_lat.pub_cmd()

            rate.sleep()