#!/usr/bin/env python


import rospy
#from utils import *

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates 

rospy.init_node('remove_obj')
del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
data = rospy.wait_for_message('/gazebo/model_states', ModelStates)
data = [d for d in data.name if 'task' in d]
print data, len(data)

for name in data:
    print name
    del_model(name)
        
    

    
