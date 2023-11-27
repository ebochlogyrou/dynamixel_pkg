from helper_functions_symb import pseudoInverseMat

from sympy import symbols, lambdify, pi, cos, sin, asin, acos, sqrt
import numpy as np
import time
import matplotlib.pyplot as plt



#Here the Calculated Jacobians get implepented (they have been implemented beforehand) thus Analytical_Jac(q1, q2)
#also the angles phi, theta (phi ,theta (q1, q2))

q1, q2 = symbols('q1 q2')


Jac00 = 0.838309308691968*(-1.0*sin(q1)*sin(q2) + 0.707106781186548*cos(q1)*cos(q2) + 0.707106781186548*cos(q1))*(1.0*cos(q2) - 0.25*cos(2*q2) + 1.25)/(sqrt((0.82842712474619*cos(q1) - 0.17157287525381*cos(q1 - q2) + cos(q1 + q2))**2/(1.0*cos(q2) - 0.25*cos(2*q2) + 1.25))*(0.8*cos(q2) - 0.2*cos(2*q2) + 1)**(3/2))
Jac01 = 0.838309308691968*(0.75*cos(q1) - 0.0517766952966369*cos(q1 - 2*q2) + 0.146446609406726*cos(q1 - q2) + 0.853553390593274*cos(q1 + q2) + 0.301776695296637*cos(q1 + 2*q2))/(sqrt((0.82842712474619*cos(q1) - 0.17157287525381*cos(q1 - q2) + cos(q1 + q2))**2/(1.0*cos(q2) - 0.25*cos(2*q2) + 1.25))*(0.8*cos(q2) - 0.2*cos(2*q2) + 1)**(3/2))
Jac10 = 0
Jac11 = 0.316227766016838*(-1.24126707662364e-16*sin(q2)**2 + 2.23606797749979*cos(q2) - 2.23606797749979)*sin(q2)/(sqrt(0.5*sin(q2)**2 + 1.0*cos(q2) + 1.0)*sqrt((cos(q2) - 1)**2))

phi_symb = asin(0.894427190999916*(0.707106781186548*sin(q1) - 0.146446609406726*sin(q1 - q2) + 0.853553390593274*sin(q1 + q2))/sqrt(0.8*cos(q2) - 0.2*cos(2*q2) + 1))
theta_symb = asin(0.790569415042095*sqrt(0.8*cos(q2) - 0.2*cos(2*q2) + 1))


numeric_Jac00 = lambdify((q1, q2), Jac00, modules='numpy')
numeric_Jac01 = lambdify((q1, q2), Jac01, modules='numpy')
numeric_Jac10 = lambdify((q1, q2), Jac10, modules='numpy')
numeric_Jac11 = lambdify((q1, q2), Jac11, modules='numpy')

phi_num = lambdify((q1, q2), phi_symb, modules='numpy')
theta_num = lambdify((q1, q2), theta_symb, modules='numpy')







q = np.array([0.01 ,0.01]) #initial guess (can be last position of the nozzle)
chi_des = np.array([1.0, np.pi/4]) #


##tuning Parameters for the numerical Calculation in the while-loop
it_max = 100000
alpha = 0.9 #still needs to be tested for different possible configurations
damping = 0.01
acceptable_chi_err = 1/180*np.pi

alpha_time = np.empty(1000)




start_time = time.time()




for it in range(it_max):

    #gets the now estimated values for q
    q1_val, q2_val = q

    #calculates the numerical Values for chi and the Jacobian depending on the estimated 11, q2
    chi_now = np.array([phi_num(q1_val, q2_val), theta_num(q1_val, q2_val) ])

    numJac = np.array([
    [numeric_Jac00(q1_val, q2_val), numeric_Jac01(q1_val, q2_val)],
    [numeric_Jac10(q1_val, q2_val), numeric_Jac11(q1_val, q2_val)]
    ])


    numJac_pinv = pseudoInverseMat(numJac, damping)


    chi_err = chi_des - chi_now

    A = ((alpha*numJac_pinv) @ chi_err).T

    q = q + A 


    if np.all(np.abs(chi_err) < acceptable_chi_err):
        break   

    
    






print("--- %s seconds ---" % (time.time() - start_time))
print("Iteration",it)

chi_now = np.array([phi_num(q[0], q[1]), theta_num(q[0], q[1]) ])

print(chi_now)



#print(simplify(Jac_symb)) simplify insane langsam