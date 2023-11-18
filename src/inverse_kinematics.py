#!/usr/bin/env python3

#this is the ROS integration of the inverse Kinematics_file for testing inverse_Kinematic_test.py is used
import rospy
import numpy as np

from std_msgs.msg import Int32MultiArray 
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk_examples.msg import SetPosition 

class distributor: # takes an array which has the size of the number of dynamixels and  sends it as individual messages to the read_write_node.py
    def __init__(self):
        pub_topic_name = "rotation_angles" 
        sub_topic_name = "normal_vector" #topic comming from the deg_to_clicks

        self.pub = rospy.Publisher(pub_topic_name, SetPosition, queue_size=10) 
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Float32MultiArray, self.callback)
        self.set_individ_pose_msg = SetPosition()

    
    def inverse_kinematics_n_to_q(self, msg): #calculates the rotatio angles of the swivle nozle (for two angles)

        it = 0
        max_it = 0
        damping = 0.001 #for inverse kinematics
        alpha = 0.5

        


        
        
        for index, element in enumerate(msg.data):  #tranverses through message array
            self.set_individ_pose_msg.id = index + 1
            #because matrices start from 0 index
            self.set_individ_pose_msg.position = element
            self.pub.publish(self.set_individ_pose_msg) 
            #rospy.sleep(0.2)

        

         


if __name__ == '__main__': 
    node_name = "publish_bulk_commands_class"
    rospy.init_node(node_name)
    distributor() 
    rospy.spin()