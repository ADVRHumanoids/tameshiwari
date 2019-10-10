# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 14:56:05 2019

@author: Paul Janssen
"""

from casadi import collocation_points
import numpy as np

class ReturnValue:
  def __init__(self, B, C, D):
     self.B = B
     self.C = C
     self.D = D

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

    return ReturnValue(B,C,D)