"""
    @author Busra Akdag
    @date 15.01.2023
    @brief This application has been created with a simple approach to run the transfer function and other
            properties of the second-order system with the given oscillation value.
            P_type controller has been designed for the kp value of the system and
            step info response of the system was given via the application.

            It can be developed for complex and higher order systems.

"""

import math
from sympy import *
import numpy as np
from control import *
import matplotlib.pyplot as plt
from numpy import log as ln
import cdk
from control import step_info, TransferFunction
from control.pzmap import pzmap
import matplotlib.pyplot as plt

class ptype():

    def __init__(self,Gss,os): #Mathemathical model of system must depend on 's' and the solution required oscillation value
        s = Symbol('s')
        kp = Symbol('kp')
        wn = Symbol('wn')
        Fss = kp #kp values of system

        zeta = -log(os) / math.sqrt((math.pi) ** 2 + log(os) ** 2)

        pds = s ** 2 + 2 * zeta * wn * s + wn ** 2 #denumerator of desired polynom
        pds = Poly(pds, s)
        pds_den = pds.all_coeffs() #coefficients of desired polynom

        Tss = (Gss * Fss) / (1 + (Fss * Gss)) #system with kp value
        Tss = cancel(Tss)
        num, den = fraction(Tss)
        num = Poly(num, s).all_coeffs()
        den = Poly(den, s).all_coeffs()

        eq_list = []

        for i in range(1, len(den)):
            eq_list.append(Eq(den[i], pds_den[i]))

        sol = nonlinsolve(eq_list, [kp, wn])  # NonLinear Solution
        solution_list = []

        for i in sol:
            solution_list.append(i)

        solutions = []
        for i in range(1):
            solutions.append(solution_list[0][i])

        print('kp value : ', solutions[0])


        Fs = np.array(solutions[0], ndmin=1, dtype=np.dtype(float))

        Gss = cancel(Gss)
        plant_num, plant_den = fraction(Gss)
        plant_num = Poly(plant_num, s).all_coeffs()
        plant_den = Poly(plant_den, s).all_coeffs()
        plant_den = np.array(plant_den, ndmin=1, dtype=np.dtype(float))
        plant_num = np.array(plant_num, ndmin=1, dtype=np.dtype(float))
        Gs = tf(plant_num,plant_den) #transfer function of system

        Ts = feedback(Gs * Fs, 1)
        t, y = step_response(Ts)

        S = step_info(Ts)
        for k in S:
            print(f"{k}: {S[k]}")

        print('Transfer Function of System ', Ts)

        p = Ts.pole()
        p = p.tolist()

        if len(p) > 0 :
            print('Poles of System = ', Ts.pole())
        else:
            print('The System has not poles')

        z = Ts.zero()
        z = z.tolist()

        if len(z) > 0 :
            print('Zeros of The System = ', Ts.zero())
        else:
            print('The System has not zeros')


        plt.plot(t, y)
        plt.title('Step Info Response of System')
        plt.grid()
        plt.show()

