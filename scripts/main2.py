#!/usr/bin/env python

import rospy
import tf
import time

import smach
from smach_ros import SimpleActionState, ConditionState, IntrospectionServer
import dynamic_reconfigure.client

from move_base_msgs.msg import MoveBaseGoal
rospy.init_node("smach_test")

from utils import *

rgbd = RGBD()

class TFBroadCaster:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.to_tf = []
        self.from_tf = []

    def loop(self, event):
        if len(self.x) == 0:
            return
        for x, y, z, roll, pitch, yaw, to_tf, from_tf in zip(self.x, self.y,
                                                             self.z, self.roll,
                                                             self.pitch, self.yaw,
                                                             self.to_tf, self.from_tf):
            # print x, y, z, roll, pitch, yaw, to_tf, from_tf
            rgbd._br.sendTransform(
                (x, y, z), tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                rospy.Time.now(), to_tf, from_tf)

    def add(self,x,y,z,roll,pitch,yaw,to_tf,from_tf):
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.yaw.append(yaw)
        self.to_tf.append(to_tf)
        self.from_tf.append(from_tf)

class UpdateObstacle(smach.State):
    def __init__(self, max_obstacle_height):
        smach.State.__init__(self, outcomes=['success'])
        self.max_obstacle_height = max_obstacle_height
    def execute(self, userdata):
        client = dynamic_reconfigure.client.Client('/move_base/local_costmap/obstacles',
                                                   timeout=30)
        client.update_configuration({"max_obstacle_height":self.max_obstacle_height})
        rospy.loginfo('update dynamic_reconfigure')

        return 'success'


class FindApple(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],
                             input_keys=['tf_x', 'tf_y', 'tf_z'],
                             output_keys=['tf_x', 'tf_y', 'tf_z'])
    def execute(self, userdata):
        move_head_tilt(-0.5)
        rgbd.set_h(125, 255)
        while not rospy.is_shutdown():
            region = rgbd.get_region()
            x, y, z = rgbd.get_xyz()
            if region is None:
                rospy.loginfo('waiting for correcto data is comming')
                continue
            elif x == 0 and y == 0 and z == 0:
                rospy.loginfo('waiting for correcto data is comming')
                continue
            else:
                rospy.loginfo('correct data is comming')
                userdata.tf_x = x
                userdata.tf_y = y
                userdata.tf_z = z
                print userdata
                return 'success'
    
def construct_sm():

    broadcaster = TFBroadCaster()
    tf_cast_loop = rospy.Timer(rospy.Duration(0.1), broadcaster.loop)
    
    sm = smach.StateMachine(outcomes = ['success','failure'])
    sm.userdata.nums = range(5)
    
    sm.userdata.tf_x = 0
    sm.userdata.tf_y = 0
    sm.userdata.tf_z = 0

    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo('wait for move_base server')
    move_base_client.wait_for_server()

    with sm:

        @smach.cb_interface(outcomes=['success'])
        def start_cb(ud):
            rospy.loginfo('start the test')
            move_arm_neutral()
            return 'success'
        smach.StateMachine.add('START', smach.CBState(start_cb), 
                               transitions = {'success':'ITERATOR'})

        iterator = smach.Iterator(outcomes = ['success','failure'],
                                  input_keys = ['tf_x', 'tf_y', 'tf_z'],
                                  output_keys = [],
                                  it = lambda: range(0, len(sm.userdata.nums)),
                                  it_label = 'index',
                                  exhausted_outcome = 'success')
        with iterator:
            
            container_sm = smach.StateMachine(input_keys = ['tf_x', 'tf_y', 'tf_z',],
                                              outcomes = ['continue', 'failure'])
            with container_sm:

                smach.StateMachine.add('NAVOBSTACLE', UpdateObstacle(0.5),
                                       transitions = {'success':'Move2Shelf'})
                
                goal = MoveBaseGoal()
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = 2.2
                goal.target_pose.pose.position.y = 4.1
                goal.target_pose.pose.position.z = 0
                quaternion = tf.transformations.quaternion_from_euler(0,0,1.57)
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]
                print goal
                smach.StateMachine.add('Move2Shelf',
                                       SimpleActionState('/move_base', MoveBaseAction, goal=goal),
                                       transitions={'succeeded':'FindApple',
                                                    'preempted':'failure',
                                                    'aborted':'failure'})
                
                smach.StateMachine.add('FindApple', FindApple(),
                                       transitions = {'success':'BROADCASTTF'})
                        

                @smach.cb_interface(input_keys=['tf_x', 'tf_y', 'tf_z'],
                                    outcomes=['success'])
                def broadcast_tf_cb(ud):
                    print ud.tf_x, ud.tf_y, ud.tf_z
                    #broadcast head_rgbd_sensor to apple position
                    broadcaster.add(ud.tf_x, ud.tf_y, ud.tf_z, 0.5, 0, 0,
                                    'apple', 'head_rgbd_sensor_rgb_frame')

                    approach_x = ud.tf_x
                    tate = 0.1 * math.tan(0.5)
                    approach_y = ud.tf_y + tate
                    approach_z = ud.tf_z - 0.1
                    broadcaster.add(approach_x, approach_y, approach_z, 0.5, 0, 0,
                                    'approach_p', 'head_rgbd_sensor_rgb_frame')
                    
                    hand_to_map = get_relative_coordinate('map', 'approach_p')
                    broadcaster.add(hand_to_map.translation.x,
                                    hand_to_map.translation.y,
                                    hand_to_map.translation.z,
                                    -1.57, -1.57, 0,
                                    'map_approach_p', 'map')

                    return 'success'
                
                smach.StateMachine.add('BROADCASTTF', smach.CBState(broadcast_tf_cb), 
                                       transitions = {'success':'GRASPOBSTACLE'})
                
                smach.StateMachine.add('GRASPOBSTACLE', UpdateObstacle(0),
                                       transitions = {'success':'MOVE2CENTER'})

                @smach.cb_interface(outcomes=['success'])
                def move_to_center_cb(ud):
                    rospy.loginfo('enterenterenterenter')
                    while not rospy.is_shutdown():
                        relative_p = get_relative_coordinate2("hand_palm_link", "map_approach_p")
                        duration = (rospy.Time.now() - relative_p.header.stamp).to_sec()
                        print 'duration', duration
                        if duration < 1:
                            break
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "base_footprint"
                    goal.target_pose.pose.position.x = 0
                    goal.target_pose.pose.position.y = -relative_p.transform.translation.y
                    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
                    navclient.send_goal(goal)
                    navclient.wait_for_result()
                    state = navclient.get_state()
                    
                    return 'success'
                
                smach.StateMachine.add('MOVE2CENTER', smach.CBState(move_to_center_cb), 
                                       transitions = {'success':'GRASPOBJECT'})
                
                @smach.cb_interface(outcomes=['success'])
                def grasp_object_cb(ud):
                    add_collision_scene(frame_id='map_approach_p')
                    result = move_hand(1)
                    print "hand close result is ", result
                    while not rospy.is_shutdown():
                        # trans = get_relative_coordinate("map", "approach_p")
                        trans = get_relative_coordinate("map", "map_approach_p")

                        result = move_wholebody_ik(trans.translation.x,
                                                   trans.translation.y,
                                                   trans.translation.z, -90, 0, -90, 'map')
                        print result
                        if result == True:
                            break
                    time.sleep(2)

                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "base_footprint"
                    goal.target_pose.pose.position.x = 0.15
                    goal.target_pose.pose.position.y = 0
                    goal.target_pose.pose.position.z = 0
                    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
                    # goal.target_pose.header.frame_id = "hand_palm_link"
                    # goal.target_pose.pose.position.x = 0.15
                    # goal.target_pose.pose.position.y = 0
                    # goal.target_pose.pose.position.z = 0
                    # goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 0)
                    navclient.send_goal(goal)
                    navclient.wait_for_result()
                    state = navclient.get_state()
                    print 'nav_state', state
                    time.sleep(3)
                    move_hand(0)
                    time.sleep(3)

                    goal.target_pose.pose.position.x = -1
                    navclient.send_goal(goal)
                    navclient.wait_for_result()
                    state = navclient.get_state()
                    print 'back nav_state', state
                    
                    while not rospy.is_shutdown():
                        result = move_arm_init()
                        if result is True:
                            break
                    print 'move to arm init position !!!!!!!!!!'
                    
                    return 'success'

                smach.StateMachine.add('GRASPOBJECT', smach.CBState(grasp_object_cb), 
                                       transitions = {'success':'ORIGINOBSTACLE'})
                
                smach.StateMachine.add('ORIGINOBSTACLE', UpdateObstacle(0.5),
                                       transitions = {'success':'Move2Origin'})

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
                                       transitions={'succeeded':'OPENGRIPPER',
                                                    'preempted':'failure',
                                                    'aborted':'failure'})
                
                @smach.cb_interface(outcomes=['success'])
                def open_gripper_cb(ud):
                    result =  move_hand(1)
                    rospy.loginfo("Gripper open result is" + str(result))
                    return 'success'

                smach.StateMachine.add('OPENGRIPPER', smach.CBState(open_gripper_cb), 
                                       transitions = {'success':'continue'})
                
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
