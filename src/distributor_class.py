#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray 
from dynamixel_sdk_examples.msg import SetPosition 

class distributor: # takes an array which has the size of the number of dynamixels and sends it as individual messages to the read_write_node.py
    def __init__(self):
        pub_topic_name = "set_position" 
        sub_topic_name = "motor/position" #topic comming from the deg_to_clicks

        self.pub = rospy.Publisher(pub_topic_name, SetPosition, queue_size=10) 
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Int32MultiArray, self.callback)
        self.set_individ_pose_msg = SetPosition()

    
    def callback(self, msg): 
        for index, element in enumerate(msg): 
            self.set_individ_pose_msg.id = index + 1 
            self.set_individ_pose_msg.position = element
            self.pub.publish(self.set_individ_pose_msg) 
         


if __name__ == '__main__': 
    node_name = "publish_bulk_commands_class"
    rospy.init_node(node_name)
    distributor() 
    rospy.spin()


