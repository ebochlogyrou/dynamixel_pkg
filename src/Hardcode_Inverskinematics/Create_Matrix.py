import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def make_vector(q1_deg,q2_deg):
    # Define variables
    e1 = np.pi / 4
    e2 = np.pi / 4
    
    #180°-pi
    #x- y.px
    q1=q1_deg*np.pi/180
    q2=q2_deg*np.pi/180
    # Define transformation matrices
    C_01 = np.array([
        [np.cos(q1), 0, np.sin(q1)],
        [0, 1, 0],
        [-np.sin(q1), 0, np.cos(q1)]
    ])

 
    C_12 = np.array([
        [np.cos(-e1), -np.sin(-e1), 0],
        [np.sin(-e1), np.cos(-e1), 0],
        [0, 0, 1]
    ])

    C_23 = np.array([
        [np.cos(q2), 0, np.sin(q2)],
        [0, 1, 0],
        [-np.sin(q2), 0, np.cos(q2)]
    ])

    C_34 = np.array([
        [np.cos(-e2), -np.sin(-e2), 0],
        [np.sin(-e2), np.cos(-e2), 0],
        [0, 0, 1]
    ])

    C_0E = np.dot(np.dot(np.dot(C_01, C_12), C_23), C_34)

    return C_0E[:,[1]]

def get_matrix():
    print('Making Matrix')
    for a1 in range(0, int(360 / step_size)):
        q1 = a1 * step_size
        for a2 in range(0, int(360 / step_size)):
            q2=a2*step_size
            y_vector=make_vector(q1,q2)          
            #Stack the vectors
            if q2==0: 
                temp_vec=y_vector 
            else:
                temp_vec=np.vstack((temp_vec,y_vector))

        if q1==0:
            val_matrix=temp_vec
        else:  
            val_matrix=np.column_stack((val_matrix,temp_vec))   
    return val_matrix

step_size=1
matrix=get_matrix()
np.save(f'Matrix_{step_size}_rad.npy', matrix)
