#!/usr/bin/env python

import rospy
import tf
import time

import smach
from smach_ros import SimpleActionState, ConditionState, IntrospectionServer

from move_base_msgs.msg import MoveBaseGoal
rospy.init_node("smach_test")

from utils import *
    
def construct_sm():
    sm = smach.StateMachine(outcomes = ['success','failure'])
    sm.userdata.nums = range(5)
    sm.userdata.counter = 0

    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo('wait for move_base server')
    move_base_client.wait_for_server()

    with sm:

        @smach.cb_interface(outcomes=['success'])
        def start_cb(ud):
            rospy.loginfo('start the test')
            return 'success'
        smach.StateMachine.add('START', smach.CBState(start_cb), 
                               transitions = {'success':'ITERATOR'})

        iterator = smach.Iterator(outcomes = ['success','failure'],
                                  input_keys = [],
                                  output_keys = [],
                                  it = lambda: range(0, len(sm.userdata.nums)),
                                  it_label = 'index',
                                  exhausted_outcome = 'success')
        with iterator:
            
            container_sm = smach.StateMachine(outcomes = ['continue', 'failure'])
            with container_sm:
                
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
                smach.StateMachine.add('Move2Shelf',
                                       SimpleActionState('/move_base', MoveBaseAction, goal=goal),
                                       transitions={'succeeded':'Move2Origin',
                                                    'preempted':'failure',
                                                    'aborted':'failure'})
                
                smach.StateMachine.add('FindApple', FindObject,
                                       transitions={'success':'',
                                                    'failure':''})

                smach.StateMachine.add('FindApple', FindObject,
                                       transitions={'success':'',
                                                    'failure':''})
                smach.StateMachine.add('FindApple', GraspObject,
                                       transitions={'success':'',
                                                    'failure':''})


                
                goal = MoveBaseGoal()
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = 2
                goal.target_pose.pose.position.y = 3
                goal.target_pose.pose.position.z = 0
                quaternion = tf.transformations.quaternion_from_euler(0,0,0)
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]
                print goal
                smach.StateMachine.add('Move2Origin',
                                       SimpleActionState('/move_base', MoveBaseAction, goal=goal),
                                       transitions={'succeeded':'continue',
                                                    'preempted':'failure',
                                                    'aborted':'failure'})


            smach.Iterator.set_contained_state('CONTAINER_STATE', 
                                               container_sm, 
                                               loop_outcomes=['continue', 'failure'])

        smach.StateMachine.add('ITERATOR', iterator,
                               transitions = {'success':'success',
                                              'failure':'failure'})
    return sm

def main():
    sm_iterator = construct_sm()
    outcome = sm_iterator.execute()
    rospy.spin()

if __name__ == "__main__":
    main()
