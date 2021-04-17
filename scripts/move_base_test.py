#!/usr/bin/env python

import rospy
from utils import *

rospy.init_node('testmove')

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "hand_palm_link"
goal.target_pose.pose.position.x = 0
goal.target_pose.pose.position.y = 1
goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
navclient.send_goal(goal)
navclient.wait_for_result()
state = navclient.get_state()
