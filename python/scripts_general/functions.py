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
import matplotlib.pyplot as plt

class ColMatrices:
    def __init__(self,d):
        self.d = d
        self.B = np.zeros(self.d+1)
        self.C = np.zeros([self.d+1,self.d+1]) 
        self.D = np.zeros(self.d+1)
        self.tau = np.append(0, collocation_points(self.d, 'legendre'))
        
        # Construct polynomial basis
        for j in range(self.d+1):
            # Construct Lagrange polynomials to get the polynomial basis at the collocation point
            p = np.poly1d([1])
            for r in range(self.d+1):
                if r != j:
                    p *= np.poly1d([1, -self.tau[r]]) / (self.tau[j]-self.tau[r])
        
            # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
            self.D[j] = p(1.0)
        
            # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
            pder = np.polyder(p)
            for r in range(self.d+1):
                self.C[j,r] = pder(self.tau[r])
        
            # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
            pint = np.polyint(p)
            self.B[j] = pint(1.0)

class RobotPose:
    def __init__(self,name=[],q=np.array([]),qdot=np.array([]),tau=np.array([]),rate=10.):
        # Make sure q, qdot, tau are matrices of N x nj
        # N is number of samples and nj is number of joints
        self.name   = name
        self.q      = q
        self.qdot   = qdot
        self.tau    = tau
        self.rate   = float(rate)
        self.nj     = 1         # default number of joints = 1
        # print self.rate
        
        # Determine the amount of time samples and final time
        if np.shape(self.q)[0] > 1:
            self.N = np.shape(self.q)[0]
            self.Tf = (self.N-1)/rate
            self.T = np.matlib.linspace(0,self.Tf,self.N)

        # Determine the number of joints dependent on q
        if np.ndim(self.q) > 1 and np.shape(self.q)[1] > 1:      
            self.nj = np.shape(self.q)[1]
        
    def interpolate(self,drate,method=''):
        # This function interpolates the known trajectory and adds more intermediate
        # points to it to have smooth motion in rviz
        # Techniques used are from Siciliano book chapter 4 Trajectory Planning

        # FIrst we compare the rate of the instance with the desired rate.
        # If the desired rate is smaller or equal than self.rate then nothing will be processed
        if self.rate <= drate and np.shape(self.qdot)[0] == np.shape(self.q)[0]:
            
            # The size of q, qdot and tau will be greater than before.
            # tau will be interpolated using zero order-hold technique
            N_new = self.Tf*drate + 1
            T_new = np.matlib.linspace(0,self.Tf,N_new)
            # print "T vector of original: %s" %self.T
            # print "T vector of new: %s" %T_new
            q_new = np.zeros([N_new,self.nj])
            qdot_new = np.zeros([N_new,self.nj])

            # Algorithm: interpolating polynomials with imposed velocities at path points
            for j in range(self.nj):
                for k in range(self.N-1):
                    qi = self.q[k,j]
                    qf = self.q[k+1,j]
                    qdoti = self.qdot[k,j]
                    qdotf = self.qdot[k+1,j]
                    ti = self.T[k]
                    tf = self.T[k+1]

                    Ar0 = [1, ti, ti**2, ti**3]
                    Ar1 = [0, 1, 2*ti, 3*ti**2]
                    Ar2 = [1, tf, tf**2, tf**3]
                    Ar3 = [0, 1, 2*tf, 3*tf**2]
                    A = np.array([Ar0,Ar1,Ar2,Ar3])
                    b = np.array([qi,qdoti,qf,qdotf])
                    x = np.linalg.solve(A,b)
                    xdot = np.polyder(x)
                    x = np.flipud(x)
                    xdot = np.flipud(xdot)
                    
                    ind = np.nonzero((T_new >= self.T[k]) & (T_new < self.T[k+1]))[0]
                    sz = np.size(ind)
                    for i in range(sz):
                        q_new[ind[i],j] = np.polyval(x,T_new[ind[i]])
                        qdot_new[ind[i],j] = np.polyval(xdot,T_new[ind[i]])
                    if k == self.N-2:
                        q_new[-1,j] = np.polyval(x,T_new[-1])
                        qdot_new[-1,j] = np.polyval(xdot,T_new[-1])
            self.q = q_new
            self.qdot = qdot_new
            self.T = T_new
            self.rate = drate
            self.N = N_new


            

        