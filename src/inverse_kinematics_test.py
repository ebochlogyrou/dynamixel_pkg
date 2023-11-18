from helper_functions.helper_inverse_kinematics import inverse_kinematics_n_to_q
import numpy as np

#for testing the inverse kinematics

############################################################################################################################
#main
if __name__ == "__main__":

    desired_normal_vector = np.array([1,0,0]) #in inertial frame(I or O in the drawing)

    q,dph = inverse_kinematics_n_to_q(desired_normal_vector)
   

    print("q ", q)
    print("kkkkk")
    print("dph ", dph)


