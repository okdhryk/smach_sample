# -*- coding:utf-8 -*-
#!/usr/bin/env python

import threading
import time

import rospy
rospy.init_node('tftest')

from utils import *
import matplotlib.pyplot as plt
import tf
import tf2_geometry_msgs
import cv2

from geometry_msgs.msg import PointStamped, PoseStamped

move_head_tilt(-0.5)
move_arm_neutral()
rgbd = RGBD()

map_x, map_y, map_z = 0,0,0
app_x, app_y, app_z = 0,0,0


class TFP(threading.Thread):
    def run(self):
        while not rospy.is_shutdown():
            rgbd._br.sendTransform(
                (x, y, z), tf.transformations.quaternion_from_euler(0.5, 0, 0),
                rospy.Time.now(),
                'example',
                'head_rgbd_sensor_rgb_frame')
            rgbd._br.sendTransform(
                (app_x, app_y, app_z), tf.transformations.quaternion_from_euler(0.5, 0, 0),
                rospy.Time.now(),
                'example2',
                'head_rgbd_sensor_rgb_frame')
            
            rgbd._br.sendTransform(
                (map_x, map_y, map_z), tf.transformations.quaternion_from_euler(-1.57, -1.57, 0),
                rospy.Time.now(),
                'map_example',
                'map')

            rgbd._br.sendTransform(
                (map_x, map_y, map_z), tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                'map_origin',
                'map')



rgbd.set_h(125, 255)
while True:
    region = rgbd.get_region()
    x,y,z = rgbd.get_xyz()
    if region is None:
        continue
    elif x == 0 and y == 0 and z == 0:
        continue
    else:
        break
print "appleposition", x, y, z
app_x = x
tate = 0.2 * math.tan(0.5)
app_y = y+tate
app_z = z-0.2

t = TFP()
t.start()
move_hand(0)

trans = get_relative_coordinate("map", "example2")
#trans = get_relative_coordinate("map", "example")
map_x = trans.translation.x
map_y = trans.translation.y
map_z = trans.translation.z
print "map", map_x, map_y, map_z

while not rospy.is_shutdown():
    relative_p = get_relative_coordinate2("hand_palm_link", "map_example")
    duration = (rospy.Time.now() - relative_p.header.stamp).to_sec()
    print 'duration', duration
    if duration < 1:
        break
print relative_p
print 'relative_time is ', relative_p.header.stamp
print "time is ", rospy.Time.now()
pointstamped = PointStamped()
pointstamped.header = relative_p.header
pointstamped.point.x = relative_p.transform.translation.x
pointstamped.point.y = relative_p.transform.translation.y
pointstamped.point.z = relative_p.transform.translation.z

print '^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^'
#movestamped = tfBuffer.transform(pointstamped , "base_footprint" , timeout = rospy.Duration(5))

#print movestamped
time.sleep(1)
#ただの座標系に変換する必要あり
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "base_footprint"
goal.target_pose.pose.position.x = 0
goal.target_pose.pose.position.y = -pointstamped.point.y
goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
navclient.send_goal(goal)
navclient.wait_for_result()
state = navclient.get_state()

print 'moved'

move_hand(1)
while not rospy.is_shutdown():
    result = move_wholebody_ik(map_x, map_y, map_z, -90, 0, -90, 'map')
    print result
    if result == True:
        break
time.sleep(1)

# goal = MoveBaseGoal()
# goal.target_pose.header.frame_id = "base_footprint"
# goal.target_pose.pose.position.x = 0.25
# goal.target_pose.pose.position.y = 0
# goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
# navclient.send_goal(goal)
# navclient.wait_for_result()
# state = navclient.get_state()

# time.sleep(1)

# move_hand(0)
# move_arm_init()

# cv2.imshow('a', region)
# cv2.waitKey()
