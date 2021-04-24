#!/usr/bin/env python

import rospy

import tf
import tf2_ros

rospy.init_node('handdistance')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

#hand_l_distal_link hand_r_distal_link

trans = tfBuffer.lookup_transform('hand_l_distal_link', 'hand_r_distal_link',
                                  rospy.Time.now(), rospy.Duration(4.0))
print trans


