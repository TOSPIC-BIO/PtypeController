from sympy import *
import pclass

s = Symbol('s')
Gss = 5/(s**2+2*s) #the mathemathical model of the system

pclass.ptype(Gss,0.03) #Executing the code with the given oscillation value
