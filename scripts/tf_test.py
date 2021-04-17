#!/usr/bin/env python

import threading

import rospy
from utils import *
import matplotlib.pyplot as plt
import tf
import cv2
rospy.init_node('tftest')
move_head_tilt(-0.5)
rgbd = RGBD()

map_x, map_y, map_z = 0,0,0
xx, yy, zz = 0,0,0

class TFP(threading.Thread):
    def run(self):
        while not rospy.is_shutdown():
            rgbd._br.sendTransform(
                (x, y, z), tf.transformations.quaternion_from_euler(0.5, 0, 0),
                rospy.Time.now(),
                'example',
                'head_rgbd_sensor_rgb_frame')
            rgbd._br.sendTransform(
                (map_x-0.1, map_y, map_z), tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                'map_example',
                'base_footprint')

            rgbd._br.sendTransform(
                (xx, yy, zz), tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                'global_example',
                'map')
while True:
    print('---------------------')
    image_data = rgbd.get_image()
    if image_data is None:
        continue
    else:
        break
rgbd.set_h(125, 255)
while True:
    region = rgbd.get_region()
    if region is None:
        continue
    else:
        break

print region
while True:
    x, y, z = rgbd.get_xyz()
    if x == 0 and y == 0 and z == 0:
        continue
    else:
        break
print x, y, z

t = TFP()
t.start()
move_hand(0)

trans = get_relative_coordinate("base_footprint", "example")
map_x = trans.translation.x
map_y = trans.translation.y
map_z = trans.translation.z

trans = get_relative_coordinate("map", "map_example")
xx = trans.translation.x
yy = trans.translation.y
zz = trans.translation.z
print xx ,yy ,zz


# move_hand(1)
# move_wholebody_ik(xx, yy, zz, 0, 0, 0, 'map')
# move_hand(0)
# move_arm_init()



# cv2.imshow('a', region)
# cv2.waitKey()
