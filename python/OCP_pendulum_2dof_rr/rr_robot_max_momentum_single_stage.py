#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#==============================================================================
#   SUMMARY:
#   This script is the first implementation of trying to maximize momentum at
#   the end-effector in cartesian space. The possibility is to choose a direction
#   in which the momentum/velocity has to be maximized. This script only takes
#   into account the first stage of the problem which is to accelerate the EE.
# 
#   OBJECTIVE:
#   The objective is to move the pendulum from a rest position to a desired
#   final position with an as large as possible EE velocity within the 
#   joint velocity limits. The input of a final EE cartesian position is not
#   mandatory, the final pose will then depend on the which local minima it
#   finds first.
# 
#   MODEL:
#   The model is an 2 DoF double pendulum, similar to the CENTAURO arm
#   inertia. The 2 actuated joints are the shoulder flexion/extension joint
#   and the elbow joint. The actuators are of type orange and yellow respec-
#   tively. Torque and velocity limits are according to Table II of the
#   CENTAURO paper [1].
# 
#   [1]: https://ieeexplore.ieee.org/document/8630605
#
#==============================================================================

#==============================================================================
#   RELEASED PACKAGES
#==============================================================================

from casadi import *
import os
import subprocess
import numpy as np
import numpy.matlib as ml
import matplotlib.pyplot as plt
import scipy.io as sio
import rospy
from datetime import datetime

#==============================================================================
#   CUSTOM PACKAGES
#==============================================================================

import functions as fn
import joint_state as js
import tameshiwari.pynocchio_casadi as pyn

# =============================================================================
#   INITIALIZATION
# =============================================================================
var_pl      = True
var_ani     = True
var_rec     = False
var_inp     = False
var_save    = False
var_h_opt   = False

# =============================================================================
#   WEIGHTENING 
# =============================================================================
#   DEFAULT VALUES
W = 1.0
W_h = 1.0

# TODO: Change the weighenings to have effect on the proper values
#   USER INPUTS
if var_inp:
    print "================ USER INPUT ================"
    usr_h_opt = raw_input("Do you want to optimze for Time? (y/N): ")
    if usr_h_opt != "":
        try:
            if usr_h_opt.lower() == 'y':
                var_h_opt = True
            elif usr_h_opt.lower() == 'n':
                var_h_opt = False
            print "The following value was entered: %s" %var_h_opt
        except ValueError:
            var_h_opt = False
            print "Automatic response, default value: %s" %var_h_opt
    else:
        var_h_opt = False
        print "Automatic response, default value: %s" %var_h_opt

    if var_h_opt:
        print "================ USER INPUT ================"
        usr_W_time = raw_input("Enter a weighting for minimizing T (0,1.0): ")
        if usr_W_time != "":
            try:
                W_h = float(usr_W_time)
                print "The following value was entered: %s" %W_h
            except ValueError:
                print "Nothing was entered, default value: %s" %W_h

    print "================ USER INPUT ================"
    usr_W = raw_input("Enter the torque reduction factor W between (0.1,1.0): ")
    if usr_W != "":
        try:
            W = float(usr_W)
            print "The following value was entered: %s" %W
        except ValueError:
            print "Nothing was entered, default value: %s" %W

    if W <= 0.1:
        W = 0.1
    elif W > 1.0:
        W = 1.0
    else:
        pass

# =============================================================================
#   PARAMETERS
# =============================================================================

#   SIMULATION PARAMETERS
N = 120
h_0 = 0.02
Tf = N*h_0
Tf_max = 10.
h_max = float(Tf_max/120)
print h_max
T = np.arange(0,Tf+h_0,h_0)
T =  T.reshape(-1,1)
iter_max = 0
rviz_rate = 30

#   INITIAL CONDITIONS (NUMERICAL)
nj = 2
nq = 3 # number of different q's --> q(t), qdot(t), qddot(t)
# q_0 = np.zeros(nj).tolist()
q_0 = np.array([np.pi*3/4,np.pi/2]).reshape(-1,2)
# q_0_vec = np.matlib.linspace(0,np.pi,N+1).reshape(-1,1)
# q_0_vec = np.hstack((q_0_vec,np.zeros([N+1,1]))) #.reshape(-1,1).flatten().tolist()
q_0_vec = np.zeros([N+1,nj])
# q_0_vec = np.ones([N+1,nj]) * q_0
qdot_0 = np.zeros(nj).tolist()
qddot_0 = np.zeros(nj).tolist()
# qddot_0 = np.ones(nj,dtype=float)* 0.5
# qddot_0 = qddot_0.tolist()

#   TERMINAL CONDITIONS (NUMERICAL)
J01_pos = [0.0, 0.0, 1.2]
offsetX = 0.0800682
offsetY = 0
offsetZ = 1.2
desX = 0
desY = 0.797
# desY = 0.65
desZ = 0
EE_pos_N = [desX+offsetX, desY+offsetY, desZ+offsetZ]
qdot_N = np.zeros(nj).tolist()
qddot_N = np.zeros(nj).tolist()

#   STATE & CONTROL BOUNDS
#   POSITION BOUNDS
lbq_deg = [-95, -20]
ubq_deg = [195, 145]
lbq = np.deg2rad(lbq_deg).tolist()
ubq = np.deg2rad(ubq_deg).tolist()
#   VELOCITY BOUNDS
ubqdot = [3.9, 6.1]
lbqdot = [x*-1 for x in ubqdot]
#   TORQUE BOUNDS
ubtau = [147., 147.]
ubtau = [x*W for x in ubtau]
lbtau = [x*-1 for x in ubtau]
#   ACCELERATION BOUNDS
ubqddot = [inf, inf]
lbqddot = [x*-1 for x in ubqddot]

# =============================================================================
#   INVERSE DYNAMICS
# =============================================================================

urdf = rospy.get_param('robot_description')
id_string = pyn.generate_inv_dyn(urdf)
invDyn = Function.deserialize(id_string)

# =============================================================================
#   FORWARD KINEMATICS
# =============================================================================

fk_string = pyn.generate_forward_kin(urdf,'EE')
forKin = Function.deserialize(fk_string)

# =============================================================================
#   JACOBIAN OF END-EFFECTOR
# =============================================================================

jacEE_string = pyn.generate_jacobian(urdf,'EE')
jacEE = Function.deserialize(jacEE_string)

# =============================================================================
#   DEBUG AREA
# =============================================================================

# print forKin
# print jacEE

# =============================================================================
#   NONLINEAR PROGRAM --> FIND A SOLUTION FOR THE OPTIMAL CONTROL INPUT
# =============================================================================
#   OPTIMIZATION VARIABLES
w = []
w0 = []
lbw = []
ubw = []
#   COST FUNCTION INITIALIZATION
J = 0
#   INEQUALITY CONSTRAINT
g = []
lbg = []
ubg = []
#   TORQUE INDEX VECTOR
tau_ind = []
cnt = 0

#   Formulate the NLP
#   minimize(tau)     sum_(k=0)^(N-1) dot(tau,tau)
#   subject to        qdot_k+1 - qdot_k - h*qddot_k = 0
#                     for k = [0,1,....,N-1]
#                     \bar(x_0) - x_0  = 0, for some initial condition
#                     \bar(x_N) - x_N  = 0, for some terminal condition
#                     lbq    <= q_k    <= ubq
#                     lbqdot <= qdot_k <= ubqdot
#                     lbtau  <= tau_k  <= ubtau 
#   Decision variable vector q has the following structure:
#   q = [x_0_0 x_1_0 u_0 x_0_1 x_1_1 u_1 ..... u_N-1 x_0_N x_1_N]
#  
#   For the single shooting method only the control input is a free variable
#   for the optimization problem. The others are variable parameters that
#   change during the iterations, they are not to be used as free variables,
#   but rather as parameter variables.

#   SECTION FOR IF TIME HAS TO BE OPTIMIZED
if var_h_opt:
    h = MX.sym('h')
    w += [h]
    w0 += [h_0]
    lbw = [0.001]
    ubw = [h_max]
else:
    h = h_0

qk = MX.sym('q_0',nj)
qdotk = MX.sym('qdot_0',nj)
qddotk = MX.sym('qddot_0',nj)
w += [qk,qdotk,qddotk]
lbw += lbq + lbqdot + lbqddot
ubw += ubq + ubqdot + ubqddot
w0 += q_0_vec[0,:].tolist() + qdot_0 + qddot_0

#   INITIAL CONDITIONS STATE VECTOR (POSITION, VELOCITY & ACCELERATION)
#   --> IN GENERAL ONLY INIITAL POSITION CAN DEVIATE FROM THE ZERO VECTOR
g += [qk,qdotk,qddotk]
# lbg += q_0_vec[0,:].tolist() + qdot_0 + qddot_0
# ubg += q_0_vec[0,:].tolist() + qdot_0 + qddot_0
lbg += np.zeros(nj*nq).tolist()
ubg += np.zeros(nj*nq).tolist()

for k in range(N):
    #   NEW NLP VARIABLE FOR THE CONTROL
    tauk = invDyn(q=qk,qdot=qdotk,qddot=qddotk)['tau']
    tau_ind += [np.shape(g)[0]*2,np.shape(g)[0]*2+1]
    g += [tauk]
    lbg += lbtau
    ubg += ubtau

    #   NORMALIZE TAU AND H
    h_norm = (1/h_max) * h
    tau_k_norm = 1/np.array(ubtau) * tauk

    #   INTEGRAL COST CONTRIBUTION
    if var_h_opt:
        L = W_h * h_norm
    else:
        L = 0
    J += L

    #   INTEGRATE EULER METHOD
    qdotk_next = qdotk + h*qddotk
    qk_next = qk + h*qdotk

    #   NEW NLP VARIABLE FOR THE CONTROL
    qk = MX.sym('q_' + str(k+1),nj)
    qdotk = MX.sym('qdot_' + str(k+1),nj)
    qddotk = MX.sym('qddot_' + str(k+1),nj)
    w += [qk,qdotk,qddotk]
    lbw += lbq + lbqdot + lbqddot
    ubw += ubq + ubqdot + ubqddot
    # w0 += q_0 + qdot_0 + qddot_0
    w0 += q_0_vec[k+1,:].tolist() + qdot_0 + qddot_0

    #   ADD INQEULITY CONSTRAINTS (ON POSITION AND VELOCITY)
    g += [qk_next - qk,qdotk_next - qdotk]
    lbg += np.zeros(nj*2).tolist()
    ubg += np.zeros(nj*2).tolist()

#   TERMINAL CONSTRAINTS:
#   TERMINAL POSITION CONSTRAINT
EE_pos_constr = False
if EE_pos_constr:
    EE_pos_k = forKin(q=qk)['ee_pos']
    EE_diff_yk = EE_pos_k[1] - EE_pos_N[1]
    EE_diff_zk = EE_pos_k[2] - EE_pos_N[2]
    EE_diff_k = vertcat(EE_diff_yk,EE_diff_zk)
    EE_dist_k = norm_2(EE_diff_k)
    g += [EE_dist_k]
    lbg += [0.]
    ubg += [0.001]          # 1 millimeter deviation allowed
#   TERMINAL VELOCITY CONSTRAINT
#   CONSTRAIN THE Z VELOCITY SUCH THAT THE EE IS GOING TO ARRIVE FROM THE TOP
jacEE_N = jacEE(q=qk)['J']
EE_vel_N = mtimes(jacEE_N,qdotk)
EE_vel_zN = EE_vel_N[2]
g += [EE_vel_zN]
lbg += [-inf]
ubg += [0]

#   TERMINAL COST 
#   IS GOING TO BE -1*EE_VEL_K
E = -1*dot(EE_vel_zN,EE_vel_zN)
J += E

# =============================================================================
#   SOLVER CREATOR AND SOLUTION
# =============================================================================

nlp = dict(f=J, x=vertcat(*w), g=vertcat(*g))
opts = {}
if iter_max > 0:
    opts["ipopt"] = {"max_iter":iter_max}
solver = nlpsol('solver','ipopt',nlp,opts)
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

# =============================================================================
#   RETRIEVE THE OPTIMAL SOLUTIONS
# =============================================================================
ind = np.eye(nq)
for i in range(nq):
    indnq = list(np.ones(nj)*ind[0,i])
    indnqdot = list(np.ones(nj)*ind[1,i])
    indnqddot = list(np.ones(nj)*ind[2,i])
    tmp = np.matlib.repmat([indnq,indnqdot,indnqddot],N+1,1).reshape((N+1)*nq*nj,1) > 0
    if i == 0:
        indexer = tmp
    else:
        indexer = np.concatenate((indexer,tmp),axis=1)

w_opt = sol['x'].full()
if var_h_opt:
    h_opt = w_opt[0]
    w_opt = w_opt[1:]
else:
    h_opt = h
Tf_opt = h_opt*N
rate_opt = int(1/h_opt)
q_opt = w_opt[indexer[:,0]].reshape(-1,nj)
qdot_opt = w_opt[indexer[:,1]].reshape(-1,nj)
qddot_opt = w_opt[indexer[:,2]].reshape(-1,nj)
# h_opt = h_0

g_opt = sol['g'].full()
tau_opt = g_opt[tau_ind].reshape(-1,nj)
tau_opt = np.vstack((tau_opt,tau_opt[-1,:]))
tau_opt[0,:] = DM.nan(1,nj).full()

#==============================================================================
#   RETRIEVE CONFIGURATION TRAJECTORY
#==============================================================================

xyz = np.zeros([N+1,3])
for j in range(N+1):
    xyz[j,:] = forKin(q=q_opt[j,:])['ee_pos'].full().reshape(-1,3)

#==============================================================================
#   CREATE SOLVERPARAM & ROBOTPOSE CLASSES, THEN SAVE TO FILE IF DESIRED
#==============================================================================

# CREATE A CLASS FOR THIS WHERE WE STORE Q,QDOT,QDDOT,TAU AND ALL THE OPTIMIZER STUFF
solver_param = fn.SolverParam(solver.stats(),N,h_opt,T,Tf,sol['f'].full())
pose = fn.RobotPose(q=q_opt,qdot=qdot_opt,qddot=qddot_opt,tau=tau_opt,rate=int(1/h_opt))

if var_save:
    solver_param.saveMat()
    pose.saveMat()

#==============================================================================
#   PRINT STATEMENTS
#==============================================================================
print "The initial time step size: h_0 = %s" %h_0
print "The initial final time: Tf_0 = %s" %Tf
print "The optimized time step size: h_opt = %s" %h_opt
print "The optimized final time: Tf_opt = %s" %Tf_opt


jacEE_opt = jacEE(q=q_opt[-1,:])['J']
# print jacEE_opt
EE_vel_opt = mtimes(jacEE_opt,qdot_opt[-1,:])
# EE_vel_opt = np.matmul(jacEE_opt,np.transpose(qdot_opt[-1,:]))
print "The optimized final EE linear velocity: %s [m/s]" %EE_vel_opt[0:3]
print "The optimized final EE rotational velocity: %s [rad/s]" %EE_vel_opt[3:]
EE_vel_zopt = EE_vel_opt[2]
print "The optimized final EE velocity along the z-axis: %s [m/s]" %EE_vel_zopt
EE_pos = forKin(q=q_opt[-1,:])['ee_pos']
print "The optimized final EE cartesian position: %s [m]" %(EE_pos - J01_pos)
print "This position is w.r.t. origin of joint 1"

#==============================================================================
#   PLOTTING THE RESULTS
#==============================================================================

if var_pl or var_save:
    pose.plot_q(show=var_pl,save=var_save,title=True,block=False,lb=lbq,ub=ubq,limits=True)
    # pose.plot_qdot(show=var_pl,save=var_save,title=True,block=False,lb=lbqdot,ub=ubqdot,limits=True)
    # pose.plot_qddot(show=var_pl,save=var_save,title=True,block=False)
    # tau_lim = np.matlib.repmat(ubtau,pose.N,1)
    # pose.plot_tau(show=var_pl,save=var_save,title=True,block=False,lb=-tau_lim,ub=tau_lim,limits=True)
    # pose.plot_joint(joint=1,nq=2,show=False,save=False,title=True,block=False)
    # pose.plot_joint(joint=2,nq=2,show=False,save=False,title=True,block=False)

    #PLOT THE EE POSITION OF THE SIMULATION AND THE DESIRED POSITION


    if var_pl:
        plt.show()
    
#==============================================================================
#   ANIMATING THE RESULTS WITH RVIZ
#   HAS TO HAPPEN AFTER PLOTTING AND SAVING BECAUSE IT OVERWRITES THE REAL POSE
#==============================================================================

if var_ani:
    pose.interpolate(rviz_rate)
    print len(pose.q)
    if var_rec:
        js.posePublisher(pose)
    else:
        js.posePublisher(pose)

#==============================================================================
#   DEBUGGING AREA
#==============================================================================



#==============================================================================
#   KEEPING PLOTS OPEN
#==============================================================================
# if var_pl:
#     plt.show()