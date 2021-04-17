#!/usr/bin/env python

import threading

import rospy
from utils import *
import matplotlib.pyplot as plt
import tf
import cv2
rospy.init_node('movehand')

move_head_tilt(-0.5)
rgbd = RGBD()

trans = get_relative_coordinate("map", "aaaaaa")
map_x = trans.translation.x
map_y = trans.translation.y
map_z = trans.translation.z

move_hand(1)                          #r roll g pitch   b yaw
move_wholebody_ik(map_x, map_y, map_z, -90, 0, -90, 'map')
#move_hand(0)
#move_arm_init()



# cv2.imshow('a', region)
# cv2.waitKey()
