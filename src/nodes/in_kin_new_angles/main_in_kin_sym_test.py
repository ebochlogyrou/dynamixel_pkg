from helper_functions_symb import get_inverseJacobian, getJacobian_angles_sym, get_numeric_jac, pseudoInverseMat, get_thrustvector_symbolic, getangles_sym

from sympy import symbols, lambdify
import numpy as np
import time



#Jac_symb_inv = get_inverseJacobian() does not work yet
Jac_symb = getJacobian_angles_sym()
phi_symb, theta_symb = getangles_sym()

q1, q2 = symbols('q1 q2')

phi_num = lambdify((q1, q2), phi_symb, modules='numpy')
theta_num = lambdify((q1, q2), theta_symb, modules='numpy')

it_max = 100
alpha = 0.05
damping = 0.01

q = np.array([0.01 ,0.01]) #initial guess
chi_des = np.array([0.0, 0.0])


Jac00 = Jac_symb[0,0]
Jac01 = Jac_symb[0,1]
Jac10 = Jac_symb[1,0]
Jac11 = Jac_symb[1,1]

numeric_Jac00 = lambdify((q1, q2), Jac00, modules='numpy')
numeric_Jac01 = lambdify((q1, q2), Jac01, modules='numpy')
numeric_Jac10 = lambdify((q1, q2), Jac10, modules='numpy')
numeric_Jac11 = lambdify((q1, q2), Jac11, modules='numpy')



start_time = time.time()

acceptable_chi_err = 1/180*np.pi



for it in range(it_max):

    q1_val, q2_val = q

    chi_now = np.array([phi_num(q1_val, q2_val), theta_num(q1_val, q2_val) ])

    numJac = np.array([
        [numeric_Jac00(q1_val, q2_val), numeric_Jac01(q1_val, q2_val)],
        [numeric_Jac10(q1_val, q2_val), numeric_Jac11(q1_val, q2_val)]
    ])

    
    numJac_pinv = pseudoInverseMat(numJac, damping)

    
    chi_err = chi_des - chi_now

    A = ((alpha*numJac_pinv) @ chi_err).T

    q = q + A 

    print("chi_err" , chi_err)
    print("q",q)

    if np.all(np.abs(chi_err) < acceptable_chi_err):
        break

print("--- %s seconds ---" % (time.time() - start_time))
print("Iteration",it)

chi_now = np.array([phi_num(q[0], q[1]), theta_num(q[0], q[1]) ])

print(chi_now)



#print(simplify(Jac_symb)) simplify insane langsam