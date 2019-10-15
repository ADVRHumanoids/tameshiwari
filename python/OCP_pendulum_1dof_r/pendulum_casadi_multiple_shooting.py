#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#

#==============================================================================
#       Released packages
#==============================================================================

from casadi import *
import os
import numpy as np
import numpy.matlib as ml
import matplotlib.pyplot as plt
import scipy.io as sio

#==============================================================================
#       Custom packages
#==============================================================================

import functions as fn
import joint_state as js

# =============================================================================
#       Pendulum dynamics for state x = [phi, omega]' control u
#       x_dot = [ omega;
#                 2*sin(phi) + u];
#
# =============================================================================

#       General Parameters
var_pl      = 0
var_ani     = 0
var_save    = 0
name        = 'results_multiple_shooting'
filename    = "%s/%s.mat" % (os.getcwd(),name)

#       Simulation Parameters
N           = 60
h           = 0.2
Tf          = N*h
T           = np.arange(0,Tf+h,h)
iter_max    = 0

#       Initial Conditions (numerical)
phi_0       = -pi
omega_0     = 0
x_0         = [phi_0,    omega_0]
u_0         = 0

#       State & Control Bounds
lb_phi      = -inf
ub_phi      = inf
lb_omega    = -pi
ub_omega    = pi
lb_u        = -1.1
ub_u        = 1.1
lb_x        = [lb_phi,   lb_omega]
ub_x        = [ub_phi,   ub_omega]

# =============================================================================
#       Integrate Dynamics --> DIRECT METHOD, FIRST INTEGRATE THEN OPTIMIZE
# =============================================================================

#       CasADi Symbolics
nx          = 2
nu          = 1
phi         = SX.sym('phi')
omega       = SX.sym('omega')
x           = vertcat(phi,omega)
u           = SX.sym('u',nu)

#       Dynamics
xdot0       = omega
xdot1       = 2*np.sin(phi) + u
xdot        = vertcat(xdot0, xdot1)

#       Objective Function
L           = phi**2 + u**2

#       This RK4 integrator is a symbolic map from the state variables and
#       control inputs to the objective function and state variables at the next
#       time instance. This function will be looped over in a later part of the
#       script.
f           = Function('f',[x,u],[xdot,L])
ODE         = Function('ODE',[x,u],[xdot],['x','u'],['xd'])
X_0         = MX.sym('X_0',2)
U           = MX.sym('U')
#       MX variable is chosen because the function are evaluated many times
X           = X_0
Q           = 0
#       The next step evaluates the function f at symbolic inputs (X,U) resulting
#       in a local evaluation of the gradient of state evaluation and the area
#       under the cost function graph.
k1, k1_q    = f(X,              U)
k2, k2_q    = f(X + (h*k1)/2,   U)
k3, k3_q    = f(X + (h*k2)/2,   U)
k4, k4_q    = f(X + (h*k3),     U)
X           = X + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
Q           = Q + (h/6)*(k1_q + 2*k2_q + 2*k3_q + k4_q)
#       Create the map from the state variables to the evaluated functions
F           = Function('F',[X_0, U],[X,Q],['x0','p'],['xf','qf'])

# =============================================================================
#       NONLINEAR PROGRAM --> FIND A SOLUTION FOR THE OPTIMAL CONTROL INPUT
# =============================================================================
q           = []
q_0         = []
lbq         = []
ubq         = []
J           = 0
g           = []
lbg         = []
ubg         = []

#==============================================================================
#       Formulate the NLP
#       minimize(x,u)     sum_(k=0)^(N-1) phi^2 + u^2
#       subject to        f(x_0,u_0) - x_1            = 0
#                         f(x_1,u_1) - x_2            = 0
#                                 \vdots
#                         f(x_(N-1),u_(N-1)) - x_N    = 0
#                         \bar(x_0) - x_0             = 0
#                         -inf    <= phi_k    <= inf
#                         -pi     <= omega_k  <= pi
#                         -1.1    <= u_k      <= 1.1
#
#       Decision variable vector q has the following structure:
#       q = [x_0_0 x_1_0 u_0 x_0_1 x_1_1 u_1 ..... u_N-1 x_0_N x_1_N]
#
#       For the single shooting method only the control input is a free variable
#       for the optimization problem. The others are variable parameters that
#       change during the iterations, they are not to be used as free variables,
#       but rather as parameter variables.
#==============================================================================

Xk          = MX.sym('X0',2)                    # Define the initial condition as a variable
q           += [Xk]                             # Fill the decision variable vector with first state elements
lbq         += lb_x                             # Filling in the lower bounds of q
ubq         += ub_x                             # Filling in the upper bounds of q
q_0         += x_0                              # Initial guess for the free variables
g           += [Xk]             
lbg         += x_0                   
ubg         += x_0 

for k in range(N):
    #       New NLP variable for the control
    Uk          = MX.sym('U_' + str(k))
    q           += [Uk]
    lbq         += [lb_u]
    ubq         += [ub_u]
    q_0         += [u_0]
    #       Integrate till the end of the interval
    Fk          = F(x0=Xk, p=Uk)
    Xk_next     = Fk['xf']
    J           = Fk['qf'] + J
    #       New NLP variable for the control   
    Xk          = MX.sym('X_' + str(k+1),2)
    q           += [Xk]
    lbq         += lb_x
    ubq         += ub_x
    q_0         += x_0
    
    #       Add inqeulity constraints
    g           += [Xk_next - Xk]
    lbg         += [0, 0]
    ubg         += [0, 0]
    #       Using += instead of .extend() because += is cheaper to compute
    
#       Create a struct of the NLP with correct allocation to CasADi. Next create
#       a nlpsolver that can solve the problem at hand
nlp         = dict(f = J, x = vertcat(*q), g = vertcat(*g))
#options     = dict('max_iter' = 20)
opts        = {}
if iter_max > 0:
    opts["ipopt"] = {"max_iter":iter_max}
solver      = nlpsol('solver','ipopt',nlp,opts)
sol         = solver(x0=q_0, lbx=lbq, ubx=ubq, lbg=lbg, ubg=ubg)
q_opt       = sol['x'].full().flatten()

indnx       = list(np.ones(nx))
indnu       = list(np.zeros(nu))
indx        = np.matlib.repmat(np.concatenate((indnx,indnu),axis=0),N,1).flatten() 
indx        = np.concatenate((indx, indnx),axis=0) > 0
indu        = np.invert(indx)
x_opt       = q_opt[indx].reshape(-1,2)
phi_opt     = x_opt[:,0]
omega_opt   = x_opt[:,1]
u_opt       = q_opt[indu]

#==============================================================================
#       Saving variables to file
#==============================================================================

results             = {}
results['N']        = N
results['h']        = h
results['T']        = T
results['Tf']       = Tf
results['x']        = x_opt
results['u']        = u_opt
results['phi']      = phi_opt
results['omega']    = omega_opt
results['t_cpu']    = ''
results['n_iter']   = solver.stats()['iter_count']
results['r_stat']   = solver.stats()['return_status']
results['ub_u']     = ub_u
results['lb_u']     = lb_u
results['fval']     = sol['f'].full()

if var_save != 0:
    sio.savemat(filename, results)

#==============================================================================
#        Plotting the results
#        Plot 1 shows the state trajectories as well as the control input
#==============================================================================

if var_pl != 0:
    plt.figure(1)
    plt.clf()
    plt.plot(T,x_opt)
    plt.step(T,vertcat(DM.nan(1), u_opt))
    plt.show()

#==============================================================================
#       Animation the joint space trajectories
#       Plot the trajectory using the 2DoF pendulum URDF 
#       with the prismatic joint at 0.3m fixed position
#==============================================================================

if var_ani !=0:
    nj      = 2
    j_name  = []
    for j in range(nj):
        tmp     = "J%02d" %(j+1)
        j_name.append(tmp)
    
    q_pl = np.zeros([N+1,2])
    q_pl[:,0] = phi_opt + np.pi
    qdot_pl = np.zeros([N+1,2])
    qdot_pl[:,0] = omega_opt
    pose = fn.RobotPose(j_name,q_pl,qdot_pl,[],int(1/h))
    pose.interpolate(30)
    js.posePublisher(pose)




print len(lbg)