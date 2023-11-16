#!/usr/bin/env python
# Software License Agreement (BSD License)

## Node subscribed to /joy topic and publishing on /turtle1/cmd_vel topic

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

def callback(data):
    
    # Define your mapping from joystick input to Twist message here
    linear_scale = 40000  # Adjust as needed
    #angular_scale = 2  # Adjust as needed

    # Create a Int32MultiArray message
    # message = Int32MultiArray()
    # message.data = [0, 0]
    message.data[0]= int(linear_scale * data.axes[1])
    message.data[1] = int(linear_scale * data.axes[4])
    message.data = [int(linear_scale * data.axes[1]), int(linear_scale * data.axes[4])]

    print(f"\n left joystick: {data.axes[1]} \n right joystick: {data.axes[4]} \n publishing {message}")

    # Publish the Int32MultiArray message to the 'motor/position' topic
    motor_pos_pub.publish(message)


if __name__ == '__main__':

    #Initialize node, topics to subscribe and publish to    
    rospy.init_node('joy_to_array_node')
    joy_sub = rospy.Subscriber('/joy', Joy, callback)   #callback is a function that get's called when a subscription event occurs
    motor_pos_pub = rospy.Publisher('/motor/position', Int32MultiArray, queue_size=10)

    rate = rospy.Rate(10) #10 Hz 

    while not rospy.is_shutdown(): 
        message = Int32MultiArray()
        message.data = [0, 0]
        motor_pos_pub.publish(message) 
        rate.sleep()

    rospy.spin()