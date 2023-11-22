#needs to have sympy, scipy installed


import numpy as np
from sympy import symbols, sqrt, asin,  diff, pi, cos, sin, lambdify

def get_T_0E_symbolic(q1,q2):

    # Define variables
    e1 = np.pi / 4
    e2 = np.pi / 4
    d = 0.075159
    

    #q1 = q[0]
    #q2 = q[1]

    #q1, q2 = symbols('q1 q2')

    # Define transformation matrices
    C_01 = np.array([
        [cos(q1), 0, sin(q1)],
        [0, 1, 0],
        [-sin(q1), 0, cos(q1)]
    ])

    o_r_01 = np.zeros((3, 1))

    C_12 = np.array([
        [cos(-e1), -sin(-e1), 0],
        [sin(-e1), cos(-e1), 0],
        [0, 0, 1]
    ])

    i_r_12 = d * np.array([[cos(pi - e1)],
                          [sin(pi - e1)],
                           [ 0]])

    C_23 = np.array([
        [cos(q2), 0, sin(q2)],
        [0, 1, 0],
        [-sin(q2), 0, cos(q2)]
    ])

    ii_r_23 = np.zeros((3, 1))
    

    C_34 = np.array([
        [cos(-e2), -sin(-e2), 0],
        [sin(-e2), cos(-e2), 0],
        [0, 0, 1]
    ])

    iii_r_34 = d * np.array([[cos(pi - e2)], 
                            [sin(pi - e2)],
                            [ 0]])

    # Create transformation matrices
    T_01 = np.vstack([np.hstack([C_01, o_r_01]), np.array([0, 0, 0, 1])])
    T_12 = np.vstack([np.hstack([C_12, i_r_12]), np.array([0, 0, 0, 1])])
    T_23 = np.vstack([np.hstack([C_23, ii_r_23]), np.array([0, 0, 0, 1])])
    T_34 = np.vstack([np.hstack([C_34, iii_r_34]), np.array([0, 0, 0, 1])])

    # Calculate the final transformation matrix
    T_0E = np.dot(np.dot(np.dot(T_01, T_12), T_23), T_34)
    C_0E = T_0E[0:3, 0:3]

    return T_0E

def get_thrustvector_symbolic(q1,q2):
    post_transform_sym = get_T_0E_symbolic(q1,q2)
    o_r_ey_sym = post_transform_sym[:3, 1]  # Assuming 1:3 corresponds to the first three rows
    
    #o_r_ey_sym = simplify(o_r_ey_sym)
    
    return o_r_ey_sym


def getangles_sym(q1,q2):
    #q1, q2, x, y, z = symbols('q1 q2 x y z') #is this needed ?

    x, y, z = symbols('x y z')

    T = get_thrustvector_symbolic(q1,q2)
    x_val = T[0]
    y_val = T[1]
    z_val = T[2]
    
    r = sqrt(x_val**2 + y_val**2 + z_val**2)
    rp = sqrt(x_val**2 + z_val**2)
    
    phi = asin(-z_val / rp)
    theta = asin(rp / r)
    
    #phi = simplify(phi)
    #theta = simplify(theta)
    
    return phi, theta


def getJacobian_angles_sym():
    q1, q2 = symbols('q1 q2')
    phi, theta = getangles_sym(q1,q2)
    
    I_J_sym = np.array([
        [diff(phi, q1), diff(phi, q2)],
        [diff(theta, q1), diff(theta, q2)]
    ])
    
   # I_J_sym = simplify(I_J_sym)
    
    return I_J_sym



def pseudoInverseMat(A, lambda_val):
    # Input: Any m-by-n matrix, and a damping factor.
    # Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    
    # Get the number of rows (m) and columns (n) of A
    m, n = A.shape

    # Compute the pseudo-inverse for both left and right cases
    if m > n:
        # Compute the left pseudoinverse.
        pinvA = np.linalg.solve(A.T @ A + lambda_val**2 * np.eye(n), A.T)
    elif m <= n:
        # Compute the right pseudoinverse.
        pinvA = np.linalg.solve(A @ A.T + lambda_val**2 * np.eye(m), A.T)
    
    return pinvA

def get_inverseJacobian():
    damping = 0.001
    I_J = getJacobian_angles_sym()
    
    # Using the pseudo-inverse (pinv) from scipy
    I_J_inv = pseudoInverseMat(I_J, damping)

    return I_J_inv

def get_numeric_jac(Jac, q1_val, q2_val):
    
    Jac00 = Jac[0,0]
    Jac01 = Jac[0,1]
    Jac10 = Jac[1,0]
    Jac11 = Jac[1,1]

    q1, q2 = symbols('q1 q2')

    numeric_Jac00 = lambdify((q1, q2), Jac00, modules='numpy')
    numeric_Jac01 = lambdify((q1, q2), Jac01, modules='numpy')
    numeric_Jac10 = lambdify((q1, q2), Jac10, modules='numpy')
    numeric_Jac11 = lambdify((q1, q2), Jac11, modules='numpy')

    numJac = np.array([
        [numeric_Jac00(q1_val, q2_val), numeric_Jac01(q1_val, q2_val)],
        [numeric_Jac10(q1_val, q2_val), numeric_Jac11(q1_val, q2_val)]
    ])

    return numJac
    

