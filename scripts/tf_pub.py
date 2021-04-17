#!/usr/bin/env python

import rospy
from utils import *
import matplotlib.pyplot as plt
import tf
import cv2
rospy.init_node('tftest')
move_head_tilt(-0.5)
rgbd = RGBD()

#map 2.25168161821 4.51517083464 0.553981894621

while not rospy.is_shutdown():
    rgbd._br.sendTransform(
        (2.25, 4.0, 0.55), tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        'aaaaaa',
        'map')

