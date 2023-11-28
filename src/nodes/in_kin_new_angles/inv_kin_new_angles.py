#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from helper_functions_symb import getJacobian_angles_sym, getangles_sym, pseudoInverseMat, cart_to_spherical_coord
from sympy import symbols, lambdify


#This Node transforms a desired chi vector (phi_des, theta_des) on the endeffector of de "2DSN" to the rotation angles of the two joints.. 


#command to publish on topic: rostopic pub /normalvector_enteffector std_msgs/Float32MultiArray "{data: [0.0, 1.0, 0.0]}"

# Subscribe to "joint_angles" topic and publish to "motor/position"

def callback(msg):   
    

    ###### Frome here to
    #would not needed to be repeated for every new chi_des, is very calculalculation intensive(slow)


    #Jac_symb_inv = get_inverseJacobian() does not work for symbolic Jacobian only for numeric
    Jac_symb = getJacobian_angles_sym()
    phi_symb, theta_symb = getangles_sym()

    Jac00 = Jac_symb[0,0]
    Jac01 = Jac_symb[0,1]
    Jac10 = Jac_symb[1,0]
    Jac11 = Jac_symb[1,1]

    q1, q2 = symbols('q1 q2')
  

    phi_num = lambdify((q1, q2), phi_symb, modules='numpy')
    theta_num = lambdify((q1, q2), theta_symb, modules='numpy')

    numeric_Jac00 = lambdify((q1, q2), Jac00, modules='numpy')
    numeric_Jac01 = lambdify((q1, q2), Jac01, modules='numpy')
    numeric_Jac10 = lambdify((q1, q2), Jac10, modules='numpy')
    numeric_Jac11 = lambdify((q1, q2), Jac11, modules='numpy')
    
    #### to here

    it = 0 
    it_max = 100
    alpha = 0.05
    damping = 0.01
    #now message is phi and theta in radians
    chi_des= np.array(msg.data, dtype=np.float32)


    #taking cartisian msg and return phi and theta - doesnt work yet 
    # sph_coord = cart_to_spherical_coord(msg.data) 
    # chi_des= np.array(sph_coord, dtype=np.float32)

    q = np.array([0.01 ,0.01]) #initial guess

    acceptable_chi_err = 1/180*np.pi
    chi_err = np.array([1,1]) #jut that it enters the while loop

    while (it < it_max) and not((abs(chi_err[0])< acceptable_chi_err )and(abs(chi_err[1])< acceptable_chi_err )):
        q1_val = q[0]
        q2_val = q[1]

        chi_now = np.array([phi_num(q1_val, q2_val), theta_num(q1_val, q2_val) ])

        numJac = np.array([
            [numeric_Jac00(q1_val, q2_val), numeric_Jac01(q1_val, q2_val)],
            [numeric_Jac10(q1_val, q2_val), numeric_Jac11(q1_val, q2_val)]
        ])
        
        numJac_pinv = pseudoInverseMat(numJac, damping)


        
        chi_err = chi_des - chi_now

        A = ((alpha*numJac_pinv) @ chi_err).T

        q = q + A

        it = it + 1


    rotation_angles_rad = q
    rotation_angles_deg = rotation_angles_rad*360/(2*np.pi)
    rotation_angles_deg = np.round(rotation_angles_deg).astype(int)


    #rotation_angles_pub = Int32MultiArray(data=rotation_angles_deg.tolist())
    rot_pub.publish(Int32MultiArray(data = rotation_angles_deg ))


if __name__ == '__main__':
    # Initialize node, topics to subscribe and publish to
    node_name = "inverse_kinematics_chi_des_to_q"
    rospy.init_node(node_name, anonymous=True)

    normal_vector_sub = rospy.Subscriber('chi_des_endeffector', Float32MultiArray, callback)
    rot_pub = rospy.Publisher('joint_angles', Int32MultiArray, queue_size=10)
    rospy.spin()