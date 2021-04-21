#!/usr/bin/env python

import rospy
import tf

import smach
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseGoal

from utils import *


class Move2Shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('Move to shelf')
        move_arm_init()
        result = move_base_goal(2.2,4.1,90)
        if result == True:
            return 'success'
        else:
            if True:
                return 'timeout'
            else:
                return 'failure'
        
class FindObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'])

    def execute(self, userdata):
        rospy.loginfo('FindObject')
        try:
            move_head_tilt(-0.5)
            return 'success'
        except:
            if True:
                return 'timeout'
            else:
                return 'failure'
    

def main():
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
        smach.StateMachine.add('Move2Shelf', Move2Shelf(),
                               transitions={'success':'FindObject',
                                            'timeout':'failure',
                                            'failure':'failure'})

        smach.StateMachine.add('FindObject', FindObject(),
                               transitions={'success':'Move2Origin',
                                            'timeout':'failure',
                                            'failure':'failure'})

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        print goal
        smach.StateMachine.add('Move2Origin',
                               SimpleActionState('/move_base', MoveBaseAction, goal=goal),
                               transitions={'succeeded':'success',
                                            'preempted':'failure',
                                            'aborted':'failure'})
        
    outcome = sm.execute()
        
if __name__ == "__main__":
    rospy.init_node("smach_test")
    move_arm_init()
    main()
    

