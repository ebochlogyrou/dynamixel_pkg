import numpy as np

#kann optimiert werden, zb dass nicht jedes mal die calculate_rotMat_and_transMat(q) aufgerufen wird





def inverse_kinematics_n_to_q(n):

    
    q0 = np.array([1/np.sqrt(2),1/np.sqrt(2)])
    it = 0
    max_it = 1000
    damping = 0.001
    alpha = 0.5

    q = q0

    C_IE_des = rotation_matrix_from_vectors(np.array([0,1,0]), n)

    print(C_IE_des)



    while it < max_it:

        C_01, C_12, C_23, C_34, C_0E, T_01, T_12, T_23, T_34, T_0E = calculate_rotMat_and_transMat(q)
        I_J = joint_to_RotJac(q)
        I_J_pinv = damped_pesudo_inverse(I_J, damping)
        #print("J_pinv")
        #print(I_J_pinv)
        C_IE = C_0E

        #calculate rotation Error
        C_err = np.dot(C_IE_des, C_IE.T)
        dph = rotMatToRotVec(C_err)

        #print("q0")
        #print(q)

        #print("I_Jpinv @ dph")
        #print(I_J_pinv @ dph)
        A = np.array((np.dot((alpha*I_J_pinv), dph)).T)

        #print("A")
        #print(A)
        #print(A[0])

        #print(q + A[0])
        q = q + A[0]

        it = it + 1

    return q, dph





def rotMatToRotVec(C):
    # Input: a rotation matrix C
    # Output: the rotational vector which describes the rotation C
    
    # Tolerance for comparison to prevent division by zero
    eps = np.finfo(float).eps

    # Compute the rotation angle
    th = np.real(np.arccos(0.5 * (C[0, 0] + C[1, 1] + C[2, 2] - 1)))

    if abs(th) < eps:
        n = np.zeros((3, 1))
    else:
        # Compute the rotation axis
        n = 1 / (2 * np.sin(th)) * np.array([[C[2, 1] - C[1, 2]],
                                             [C[0, 2] - C[2, 0]],
                                             [C[1, 0] - C[0, 1]]])

    # Compute the rotational vector
    phi = th * n

    return phi
        


def damped_pesudo_inverse(A,damping_factor):
    """Calculate the damped pseudo-inverse of a matrix."""

    U, S, Vt = np.linalg.svd(A, full_matrices=False)
    S_inv = np.diag([1 / (s + damping_factor) for s in S])
    A_damped_pseudo_inv = Vt.T @ S_inv @ U.T
    return A_damped_pseudo_inv

    


def rotation_matrix_from_vectors(A, B): #calculates the rotMatrix that alings vector A with vector B // A = np.array([0,1,0])
    # Normalize vectors
    A = A / np.linalg.norm(A)
    B = B / np.linalg.norm(B)

    # Calculate rotation axis
    axis = np.cross(A, B)

    # Calculate rotation angle
    angle = np.arccos(np.dot(A, B))

    # Construct rotation matrix
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c
    x, y, z = axis

    rotation_matrix = np.array([
        [t*x**2 + c, t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y**2 + c, t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z**2 + c]
    ])

    return rotation_matrix


def joint_to_RotJac(q):
    C_01, C_12, C_23, C_34, C_0E, T_01, T_12, T_23, T_34, T_0E = calculate_rotMat_and_transMat(q)

    T_I1 = T_01
    T_I2 = np.dot(T_I1, T_12)
    T_I3 = np.dot(T_I2, T_23)
    T_I4 = np.dot(T_I3, T_34)

    R_I1 = T_I1[0:3, 0:3]
    R_I2 = T_I2[0:3, 0:3]
    R_I3 = T_I3[0:3, 0:3]
    R_I4 = T_I4[0:3, 0:3]

    n_1 = np.array([[0],
                    [1],
                    [0]])
    n_2 = np.array([[0],
                    [0],
                    [1]])
    n_3 = np.array([[0],
                    [1],
                    [0]])
    n_4 = np.array([[0],
                    [0],
                    [1]])

    J_R = np.column_stack([R_I1 @ n_1, R_I3 @ n_3])
    #print("rotJac")
    #print(J_R)
    #J_R = np.column_stack([R_I1 @ n_1, R_I2 @ n_2, R_I3 @ n_3, R_I4 @ n_4])

    return J_R






def calculate_rotMat_and_transMat(q):

    # Define variables
    e1 = np.pi / 4
    e2 = np.pi / 4
    d = 0.075159
    

    q1 = q[0]
    q2 = q[1]

    # Define transformation matrices
    C_01 = np.array([
        [np.cos(q1), 0, np.sin(q1)],
        [0, 1, 0],
        [-np.sin(q1), 0, np.cos(q1)]
    ])

    o_r_01 = np.zeros((3, 1))

    C_12 = np.array([
        [np.cos(-e1), -np.sin(-e1), 0],
        [np.sin(-e1), np.cos(-e1), 0],
        [0, 0, 1]
    ])

    i_r_12 = d * np.array([[np.cos(np.pi - e1)],
                          [np.sin(np.pi - e1)],
                           [ 0]])

    C_23 = np.array([
        [np.cos(q2), 0, np.sin(q2)],
        [0, 1, 0],
        [-np.sin(q2), 0, np.cos(q2)]
    ])

    ii_r_23 = np.zeros((3, 1))
    

    C_34 = np.array([
        [np.cos(-e2), -np.sin(-e2), 0],
        [np.sin(-e2), np.cos(-e2), 0],
        [0, 0, 1]
    ])

    iii_r_34 = d * np.array([[np.cos(np.pi - e2)], 
                            [np.sin(np.pi - e2)],
                            [ 0]])

    # Create transformation matrices
    T_01 = np.vstack([np.hstack([C_01, o_r_01]), np.array([0, 0, 0, 1])])
    T_12 = np.vstack([np.hstack([C_12, i_r_12]), np.array([0, 0, 0, 1])])
    T_23 = np.vstack([np.hstack([C_23, ii_r_23]), np.array([0, 0, 0, 1])])
    T_34 = np.vstack([np.hstack([C_34, iii_r_34]), np.array([0, 0, 0, 1])])

    # Calculate the final transformation matrix
    T_0E = np.dot(np.dot(np.dot(T_01, T_12), T_23), T_34)
    C_0E = T_0E[0:3, 0:3]

    return C_01, C_12, C_23, C_34, C_0E, T_01, T_12, T_23, T_34, T_0E




############################################################################################################################
#main
if __name__ == "__main__":

    normal_vector = np.array([0,1,0])

    q,dph = inverse_kinematics_n_to_q(normal_vector)
   

    print("q ", q)
    print("kkkkk")
    print("dph ", dph)


