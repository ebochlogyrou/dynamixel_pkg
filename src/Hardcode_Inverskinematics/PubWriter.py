#!/usr/bin/env python3


import rospy
import numpy as np
#import Kinematics_main
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from Hardcode_Inverskinematics.Kinematics_main import get_q


#This Node transforms a desired normalvector on the endeffector of de "2DSN" to the rotation angles of the two joints.. 
#the normalvector in normalvector_endeffector has to be [x,y,z] and not [x;y;z]!

#command to publish on topic: rostopic pub /normalvector_enteffector std_msgs/Float32MultiArray "{data: [0.0, 1.0, 0.0]}"


# Subscribe to "vector/rotation" topic and publish to "motor/position"
def callback(msg):   
    desired_normal_vector= np.array(msg.data, dtype=np.float32)

    rotation_angles_deg =  get_q(desired_normal_vector)
    rotation_angles_deg = np.round(rotation_angles_deg).astype(int)


    #rotation_angles_pub = Int32MultiArray(data=rotation_angles_deg.tolist())
    rot_pub.publish(Int32MultiArray(data = rotation_angles_deg ))

if __name__ == '__main__':
    # Initialize node, topics to subscribe and publish to
    node_name = "hardcode_invers"
    rospy.init_node(node_name, anonymous=True)

    normal_vector_sub = rospy.Subscriber('normalvector_endeffector', Float32MultiArray, callback)
    rot_pub = rospy.Publisher('joint_angles', Int32MultiArray, queue_size=10)
    rospy.spin()