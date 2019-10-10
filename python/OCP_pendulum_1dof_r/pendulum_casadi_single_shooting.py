#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  2 12:50:04 2019

@author: Paul
"""

from casadi import *
import numpy as np
import numpy.matlib as ml
import matplotlib.pyplot as plt
import scipy.io as sio
import matplotlib.animation as animation

# =============================================================================
#       Pendulum dynamics for state x = [phi, omega]' control u
#       x_dot = [ omega;
#                 2*sin(phi) + u];
#
# =============================================================================

#       General Parameters
var_pl      = 0
var_save    = 0

#       Simulation Parameters
N           = 60
h           = 0.2
Tf          = N*h
T           = np.arange(0,Tf+h,h)
iter_max    = 50

#       Initial Conditions (numerical)
phi_0       = -pi
omega_0     = 0;
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
lbg_test         = []

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

#       For the single shooting method only the control input is a free variable
#       for the optimization problem. The others are variable parameters that
#       change during the iterations, they are not to be used as free variables,
#       but rather as parameter variables.
Xk          = MX(x_0)
for k in range(N):
    #       New NLP variable for the control
    Uk          = MX.sym('U_' + str(k))
    q           += [Uk]
    lbq         += [lb_u]
    ubq         += [ub_u]
    q_0         += [u_0]
    #       Integrate till the end of the interval
    Fk          = F(x0=Xk, p=Uk)
    Xk          = Fk['xf']
    J           = Fk['qf'] + J
    #       Add inqeulity constraints
    g           += [Xk]
#    lbg         += lb_x.
    lbg += lb_x
    ubg += ub_x
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

u_opt       = np.array(q_opt)
x_res       = [x_0]
x_opt       = x_0
for k in range(N):
    Fk          = F(x0 = x_res[k], p = u_opt[k])
    x_res       += [Fk['xf'].full().flatten()]
    x_tmp       = Fk['xf'].full().flatten()
    x_opt.extend(x_tmp)

x_opt       = np.array(x_opt).reshape(-1,2)
phi_opt     = x_opt[:,0]
omega_opt   = x_opt[:,1]

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

sio.savemat('/tmp/mylog.mat', results)

#==============================================================================
#        Plotting the results
#        Plot 1 shows a animation of the object in motion
#        Plot 2 shows the state trajectories as well as the control input
#==============================================================================

#plt.figure(1)
#plt.clf()
#
#fig, axs    = plt.subplots()
#fig         = plt.figure(1)
#axs         = plt.axes(xlim=(-1, 1), ylim=(-1, 1))
#
#for i in range(N+1):
#    phi         = phi_opt[k]
#    th          = np.linspace(0,2*pi,100)
#    xunit       = np.cos(th)
#    yunit       = np.sin(th)
#    axs.plot([0,np.sin(phi)],[0,np.cos(phi)], '-o', color='#000000')
#    axs.plot(xunit,yunit)
##    axs.set(xlim=(-1, 1), ylim=(-1, 1))
#    axs.set_aspect('equal', 'box')
#    plt.Circle((0,0),1)
#    plt.show()

plt.figure(2)
plt.clf()
plt.plot(T,x_opt)
plt.step(T,vertcat(DM.nan(1), u_opt))
plt.show()

























   