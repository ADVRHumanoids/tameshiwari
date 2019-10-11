# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#
#   The script contains a number of useful functions and classes that are 
#   through out the Tameshiwari repo. The script will be populated in a 
#   continous way throughout the research period.
#   


from casadi import collocation_points
import numpy as np

class CollocationData:
    def __init__(self, B, C, D):
        self.B = B
        self.C = C
        self.D = D

class RobotPose:
    def __init__(self,name=['test'],q=[],qdot=[],tau=[],rate=10):
        self.name   = name
        self.q      = q
        self.qdot   = qdot
        self.tau    = tau
        self.rate   = rate

def collocation(d):
    #   from casadi import collocation_points
    B   = np.zeros(d+1)         # Coefficients of the collocation equation
    C   = np.zeros([d+1,d+1])       # Coefficients of the continuity equation
    D   = np.zeros(d+1)         # Coefficients of the quadrature function
        
    #   Create d+1 vector containing interval collocation time stamps
    tau = np.append(0, collocation_points(d, 'legendre'))
    
    # Construct polynomial basis
    for j in range(d+1):
        # Construct Lagrange polynomials to get the polynomial basis at the collocation point
        p = np.poly1d([1])
        for r in range(d+1):
            if r != j:
                p *= np.poly1d([1, -tau[r]]) / (tau[j]-tau[r])
    
        # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
        D[j] = p(1.0)
    
        # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
        pder = np.polyder(p)
        for r in range(d+1):
            C[j,r] = pder(tau[r])
    
        # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
        pint = np.polyint(p)
        B[j] = pint(1.0)

    return CollocationData(B,C,D)