from helper_functions.helper_inverse_kinematics import inverse_kinematics_n_to_q
import numpy as np

#for testing the inverse kinematics

############################################################################################################################
#main
if __name__ == "__main__":

    desired_normal_vector = np.array([0,1,0]) #in inertial frame(I or O in the drawing)

    qrad,dph = inverse_kinematics_n_to_q(desired_normal_vector)

    qdeg = qrad*360/(2*np.pi)
    qdeg_int = np.round(qdeg).astype(int)
   

    print("qdeg ", qdeg)
    print("qrad", qrad)
    print("dph ", dph)


