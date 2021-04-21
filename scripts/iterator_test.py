#!/usr/bin/env python

import rospy
import tf
import time

import smach
from smach_ros import SimpleActionState, ConditionState, IntrospectionServer

from move_base_msgs.msg import MoveBaseGoal
rospy.init_node("smach_test")
    
def construct_sm():
    sm = smach.StateMachine(outcomes = ['success','failure'])
    sm.userdata.nums = range(5)
    sm.userdata.counter = 0

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
            
            container_sm = smach.StateMachine(outcomes = ['continue'])
            with container_sm:

                @smach.cb_interface(outcomes=['success', 'continue'])
                def even_cb(ud):
                    rospy.logerr(sm.userdata.counter)
                    time.sleep(1)
                    if sm.userdata.counter == 3:
                        sm.userdata.counter += 1
                        return 'continue'
                    else:
                        sm.userdata.counter += 1
                        return 'success'
                smach.StateMachine.add('EVEN', smach.CBState(even_cb), 
                                       transitions = {'success':'ODD',
                                                      'continue':'continue'})

                @smach.cb_interface(outcomes=['success'])
                def odd_cb(ud):
                    time.sleep(1)
                    return 'success'
                smach.StateMachine.add('ODD', smach.CBState(odd_cb), 
                                       transitions = {'success':'continue'})

            smach.Iterator.set_contained_state('CONTAINER_STATE', 
                                               container_sm, 
                                               loop_outcomes=['continue'])

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
