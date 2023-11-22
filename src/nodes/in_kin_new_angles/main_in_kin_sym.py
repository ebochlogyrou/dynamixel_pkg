from helper_functions_symb import get_inverseJacobian, getJacobian_angles_sym, get_numeric_jac

from sympy import symbols, lambdify, simplify
import numpy as np



#Jac_symb = get_inverseJacobian() does not work jet
Jac_symb = getJacobian_angles_sym()

#print(Jac_symb)

q1, q2 = symbols('q1 q2')

#numeric_JAc = lambdify((q1, q2), Jac_symb, modules='numpy')
q1_val = 0.001
q2_val = 0.001

numJac = get_numeric_jac(Jac_symb, q1_val, q2_val)


#print("numeric")
#print(numeric_JAc(q1_val,q2_val))
#print(Jac_symb[1,0])


print(numJac)


#print(simplify(Jac_symb)) simplify insane langsam