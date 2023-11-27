from helper_functions_symb import get_inverseJacobian, getJacobian_angles_sym, get_numeric_jac, pseudoInverseMat, get_thrustvector_symbolic, getangles_sym

from sympy import symbols, lambdify, simplify
import numpy as np
import time



#Jac_symb_inv = get_inverseJacobian() does not work yet
Jac_symb = simplify(getJacobian_angles_sym())
phi_symb, theta_symb = simplify(getangles_sym())

print("Jac_symb")
print(Jac_symb)

print("phi")
print(phi_symb)

print("theta")
print(theta_symb)