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

#   v3 integrates the new cas_pyn library which delivers extended functionality

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
# import rec_joint_state as js
import joint_state as js
import init_state
import tameshiwari.pynocchio_casadi as pyn
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

# =============================================================================
#   INITIALIZATION
# =============================================================================
var_pl      = True
var_ani     = True
var_rec     = False
var_inp     = False
var_save    = False
var_h_opt   = True

# if var_ani:
#     # TODO: create functionality to choose homing position, default q = 0
#     init_state.homing()

# =============================================================================
#   WEIGHTENING 
# =============================================================================
#   DEFAULT VALUES
W = 0.2
W_h = 1.0

# TODO: Change the weightenings to have effect on the proper values
# TODO: Create input function in functions.py to reduce clutter
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
M = 2           # multi-stage coefficient
N_stage = [N, 60]
N_cum = np.cumsum(N_stage)
# print N_cum
N_tot = np.sum(N_stage)
# print N_tot
Tf = 3.
h_0 = float(Tf/N)
Tf_max = 10.
h_min = 0
h_max = float(Tf_max/N)
# print h_max
T = np.arange(0,Tf+h_0,h_0)
T = T.reshape(-1,1)
iter_max = 0
rviz_rate = 30

N_tot = np.sum(N_stage)
Tf0_list = [3, 1]
Tf0_vec = np.asfarray(np.array(Tf0_list,ndmin=2))
Tf_lb = [Tf0_list[0], 0.05]
Tf_lb = np.asfarray(np.array(Tf_lb,ndmin=2))
Tf_ub = [Tf0_list[0], 3]
Tf_ub = np.asfarray(np.array(Tf_ub,ndmin=2))

h0_vec = (Tf0_vec/N_stage).flatten().tolist()
# print h0_vec
lbh = (Tf_lb/N_stage).flatten().tolist()
# print lbh
ubh = (Tf_ub/N_stage).flatten().tolist()
# print ubh

# print h0_vec[0].tolist()

#   INITIAL CONDITIONS (NUMERICAL)
nj = 2
nq = 3 # number of different q's --> q(t), qdot(t), qddot(t)
# q_0 = np.zeros(nj).tolist()
# q_0 = [30, 130]
q_0 = [0, 0]
q_0 = np.deg2rad(q_0).tolist()
print q_0
# q_0 = np.array([np.pi*3/4,np.pi/2]).reshape(-1,2).flatten().tolist()
q_0_vec = np.matlib.repmat(np.array(q_0),N_tot+1,1)
# q_0_vec = np.matlib.linspace(0,np.pi,N_tot+1).reshape(-1,1)
# q_0_vec = np.hstack((q_0_vec,np.zeros([N_tot+1,1]))) #.reshape(-1,1).flatten().tolist()
# print q_0
# print np.shape(q_0_vec)
# q_0_vec = np.zeros([N_tot+1,nj])
# print q_0_vec
# q_0_vec = np.ones([N+1,nj]) * q_0
qdot_0 = np.zeros(nj).tolist()
qddot_0 = np.zeros(nj).tolist()
# qddot_0 = np.ones(nj,dtype=float)* 0.5
# qddot_0 = qddot_0.tolist()

#   HOMING TO INITIAL POSITION
if var_ani:
    # TODO: create functionality to choose homing position, default q = 0
    init_state.homing(q_0)

#   TERMINAL CONDITIONS (NUMERICAL)
J01_pos = [0.0, 0.0, 1.2]
offsetX = 0.0800682
offsetY = 0
offsetZ = 1.2
desX = 0
desY = 0.797
# desY = 0.65
desZ = 0
# desY = 0.5
# desZ = 0.4
EE_pos_N = [desX+offsetX, desY+offsetY, desZ+offsetZ]
qdot_N = np.zeros(nj).tolist()
qddot_N = np.zeros(nj).tolist()

#   STATE & CONTROL BOUNDS
#   POSITION BOUNDS
lbq_deg = [-95, -20]
# lbq_deg = [-95, 10]
ubq_deg = [195, 145]
lbq = np.deg2rad(lbq_deg).tolist()
ubq = np.deg2rad(ubq_deg).tolist()
#   VELOCITY BOUNDS
ubqdot = [3.9, 6.1]
# ubqdot = [10, 10]
lbqdot = [x*-1 for x in ubqdot]
#   TORQUE BOUNDS
W_tau = [0.4, 0.9]
tau_lim_orange = 147.
tau_lim_yellow = 147.
tau_lim = np.array([tau_lim_orange, tau_lim_yellow],ndmin=2).transpose()
tau_lim = np.matlib.repmat(tau_lim,1,M)
ubtau = tau_lim*W_tau
lbtau = ubtau*-1
print ubtau
print lbtau
#   ACCELERATION BOUNDS
ubqddot = [100, 100]
lbqddot = [x*-1 for x in ubqddot]
# print ubqddot
# print type(ubqddot)
# print lbqddot
# print type(lbqddot)

# =============================================================================
#   INVERSE DYNAMICS
# =============================================================================

urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
id_string = kindyn.rnea()
# id_string = pyn.generate_inv_dyn(urdf)
# invDyn = Function.deserialize(id_string)
invDyn = Function.deserialize(id_string)
print invDyn
print ""

# =============================================================================
#   FORWARD KINEMATICS
# =============================================================================

# fk_string = pyn.generate_forward_kin(urdf,'EE')
fk_string = kindyn.fk('EE')
forKin = Function.deserialize(fk_string)
print fk_string
print ""

# =============================================================================
#   JACOBIAN OF END-EFFECTOR
# =============================================================================

# jacEE_string = pyn.generate_jacobian(urdf,'EE')
jacEE_string = kindyn.jacobian('EE')
jacEE = Function.deserialize(jacEE_string)
print jacEE_string
print ""

# =============================================================================
#   JOINT-SPACE INERTIA MATRIX
# =============================================================================

B_string = kindyn.crba()
B_js = Function.deserialize(B_string)

# =============================================================================
#   DEBUG AREA
# =============================================================================

print forKin
print jacEE
print B_js

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

#   SECTION FOR IF TIME HAS TO BE OPTIMIZED
h = MX.sym('h_stage_1')
w += [h]
w0 += [h0_vec[0]]
lbw += [lbh[0]]
ubw += [ubh[0]]
# if var_h_opt:
#     h = MX.sym('h')
#     w += [h]
#     w0 += [h_0]
#     lbw = [0.001]
#     ubw = [h_max]
# else:
#     h = h_0

qk = MX.sym('q_0',nj)
qdotk = MX.sym('qdot_0',nj)
qddotk = MX.sym('qddot_0',nj)
w += [qk,qdotk,qddotk]
lbw += lbq + lbqdot + lbqddot
ubw += ubq + ubqdot + ubqddot
w0 += q_0_vec[0,:].tolist() + qdot_0 + qddot_0

#   INITIAL CONDITIONS STATE VECTOR (POSITION, VELOCITY & ACCELERATION)
#   --> IN GENERAL ONLY INIITAL POSITION CAN DEVIATE FROM THE ZERO VECTOR
# g += [qk,qdotk,qddotk]
# lbg += np.zeros(nj*nq).tolist()
# ubg += np.zeros(nj*nq).tolist()
g += [qk,qdotk]
lbg += q_0 + np.zeros(nj).tolist()
ubg += q_0 + np.zeros(nj).tolist()
# g += [qk,qdotk]
# lbg += np.zeros(nj*2).tolist()
# ubg += np.zeros(nj*2).tolist()
factor = np.ones(N_stage[0])
steps = 10
factor[-steps:] = np.matlib.linspace(1,0,steps)
for Mi in range(M):
    if Mi == 0:
        for k in range(N_stage[Mi]):
            #   ADD EXCLUSION OF CONFIGURATION SPACE BEFORE IMPACT
            # EE_pos_k = forKin(q=qk)['ee_pos']
            # EE_diff_yk = EE_pos_k[1] - EE_pos_N[1]
            # EE_diff_zk = EE_pos_k[2] - EE_pos_N[2]
            # EE_diff_k = vertcat(EE_diff_yk,EE_diff_zk)
            # EE_dist_k = norm_2(EE_diff_k)
            # g += [EE_dist_k]
            # absolutebound = 0.3
            # lowerbound = factor[k]*absolutebound
            # lbg += [lowerbound]
            # ubg += [inf]

            #   NEW NLP VARIABLE FOR THE CONTROL
            tauk = invDyn(q=qk,v=qdotk,a=qddotk)['tau']
            # tau_ind += [np.shape(g)[0]*2,np.shape(g)[0]*2+1]
            tau_ind += [len(lbg),len(lbg)+1]
            g += [tauk]
            lbg += lbtau[:,Mi].tolist()
            ubg += ubtau[:,Mi].tolist()

            #   NORMALIZE TAU AND H
            # h_norm = (1/ubh[0]) * h
            # tau_k_norm = 1/np.array(ubtau) * tauk

            #   INTEGRAL COST CONTRIBUTION
            if var_h_opt:
                # L = W_h * h**2
                # L = h*dot(tauk,qdotk)*0.001
                L = 0
                # L = -h*dot(tauk,tauk)*
            else:
                L = 0
            J += L

            #   INTEGRATE EULER METHOD
            qdotk_next = qdotk + h*qddotk 
            qk_next = qk + h*qdotk + 0.5*qddotk*h**2

            #   NEW NLP VARIABLE FOR THE CONTROL
            qk = MX.sym('q_' + str(k+1),nj)
            qdotk = MX.sym('qdot_' + str(k+1),nj)
            qddotk = MX.sym('qddot_' + str(k+1),nj)
            w += [qk,qdotk,qddotk]
            lbw += lbq + lbqdot + lbqddot
            ubw += ubq + ubqdot + ubqddot
            w0 += q_0_vec[k+1,:].tolist() + qdot_0 + qddot_0

            #   ADD INQEULITY CONSTRAINTS (ON POSITION AND VELOCITY)
            g += [qk_next - qk,qdotk_next - qdotk]
            lbg += np.zeros(nj*2).tolist()
            ubg += np.zeros(nj*2).tolist()


        #   TERMINAL CONSTRAINTS:
        #   TERMINAL POSITION CONSTRAINT
        EE_pos_constr = True
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
        jacEE_N_ts = jacEE_N[1:3,:] # Task space jacobian
        EE_vel_N = mtimes(jacEE_N,qdotk)
        EE_vel_zN = EE_vel_N[2]
        EE_vel_N_ts = mtimes(jacEE_N_ts,qdotk) # Task space velocity vector
        
        EE_vel_z_constr = True
        if EE_vel_z_constr:
            g += [EE_vel_zN]
            lbg += [-inf]
            ubg += [0]

        #   MAXIMUM ANGULAR VELOCITY
        # g += [qdotk]
        # lbg += lbqdot
        # ubg += lbqdot

        #   TERMINAL COST 
        #   IS GOING TO BE -1*EE_VEL_K
        # E = -1*dot(EE_vel_zN,EE_vel_zN)
        # J += E

        #   TERMINAL COST --> MOMENTUM MAXIMIZATION
        B_N = B_js(q=qk)['B']
        # Lambda_ee_N = inv(mtimes(mtimes(jacEE_N,inv(B_N)),jacEE_N.T))
        # Lambda_ee_N = inv(mtimes(jacEE_N,mtimes(inv(B_N),jacEE_N.T)))
        Lambda_ee_N = mtimes(mtimes(pinv(jacEE_N).T,B_N),pinv(jacEE_N)) # --> seems to work but is not correct to use
        # Lambda_ee_N = inv(mtimes(jacEE_N_ts,mtimes(inv(B_N),jacEE_N_ts.T)))
        h_ee_N = mtimes(Lambda_ee_N,EE_vel_N)
        # h_ee_N_ts = mtimes(Lambda_ee_N,EE_vel_N_ts)
        h_ee_zN = h_ee_N[2]
        # h_ee_zN_ts = h_ee_N_ts[1]
        # h_ee_yN_ts = h_ee_N_ts[0]
        h_ee_yN = h_ee_N[1]
        # h_ee_linN = h_ee_N[0:3]
        E = -dot(h_ee_zN,h_ee_zN)
        # E = -dot(h_ee_N,h_ee_N)
        # E = -dot(h_ee_linN,h_ee_linN)
        # E = -dot(h_ee_yN,h_ee_yN) + dot(h_ee_zN,h_ee_zN)
        # E = -dot(h_ee_yN_ts,h_ee_yN_ts)
        # E = -dot(h_ee_zN_ts,h_ee_zN_ts)
        # E = -mtimes(h_ee_linN.T,h_ee_linN)
        J += E

    else:
        h = MX.sym('h_stage_2')
        w += [h]
        w0 += [h0_vec[1]]
        lbw += [lbh[1]]
        ubw += [ubh[1]]

        for k in range(N_stage[Mi-1],N_stage[Mi]+N_stage[Mi-1]):
            #   NEW NLP VARIABLE FOR THE CONTROL
            tauk = invDyn(q=qk,v=qdotk,a=qddotk)['tau']
            # print tauk
            # tau_ind += [np.shape(g)[0]*2-2,np.shape(g)[0]*2-1]
            tau_ind += [len(lbg),len(lbg)+1]
            g += [tauk]
            lbg += lbtau[:,Mi].tolist()
            ubg += ubtau[:,Mi].tolist()

            #   NORMALIZE TAU AND H
            # h_norm = (1/ubh[1]) * h
            # tau_k_norm = 1/np.array(ubtau) * tauk

            #   INTEGRAL COST CONTRIBUTION
            if var_h_opt:
                if k > N_cum[0]+2:
                    # L = W_h * h**2 + h*dot(qdotk,qdotk)
                    L = h*dot(qdotk,qdotk)
                    # L = W_h * h**2
                    # L = W_h * h**2 + sum1(qdotk)
                else:
                    # L = W_h * h**2
                    L = h*dot(qdotk,qdotk)
            else:
                # L = dot(qk,qk)
                L = 0
                # L = dot(qdotk,qdotk)
            J += L

            #   INTEGRATE EULER METHOD
            qdotk_next = qdotk + h*qddotk 
            qk_next = qk + h*qdotk  + 0.5*qddotk*h**2

            #   NEW NLP VARIABLE FOR THE CONTROL
            qk = MX.sym('q_' + str(k+1),nj)
            qdotk = MX.sym('qdot_' + str(k+1),nj)
            qddotk = MX.sym('qddot_' + str(k+1),nj)
            w += [qk,qdotk,qddotk]
            lbw += lbq + lbqdot + lbqddot
            ubw += ubq + ubqdot + ubqddot
            w0 += q_0_vec[k+1,:].tolist() + qdot_0 + qddot_0

            # L = dot(qk-qk_next,qk-qk_next)
            # # L = dot(qdotk,qdotk)
            # J += L

            #   ADD INQEULITY CONSTRAINTS (ON POSITION AND VELOCITY)
            g += [qk_next - qk,qdotk_next - qdotk]
            lbg += np.zeros(nj*2).tolist()
            ubg += np.zeros(nj*2).tolist()

        #   TERMINAL JOINT-SPACE VELOCITY CONSTRAINT
        g += [qdotk]
        #   Relaxation of terminal velocity
        # lbter = [-0.001, -0.001]
        # ubter = [0.001, 0.001]
        # lbg += lbter
        # ubg += ubter
        lbg += np.zeros(2).tolist()
        ubg += np.zeros(2).tolist()
        #   TERMINAL COST 
        #   NO TERMINAL COST FUNCTION IS DESIRED

# =============================================================================
#   SOLVER CREATOR AND SOLUTION
# =============================================================================

nlp = dict(f=J, x=vertcat(*w), g=vertcat(*g))
opts = {}
if iter_max > 0:
    opts["ipopt"] = {"max_iter":iter_max}
solver = nlpsol('solver','ipopt',nlp,opts)
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
# print w

# =============================================================================
#   RETRIEVE THE OPTIMAL SOLUTIONS
# =============================================================================
# ind = np.eye(nq)
# for i in range(nq):
#     indnq = list(np.ones(nj)*ind[0,i])
#     indnqdot = list(np.ones(nj)*ind[1,i])
#     indnqddot = list(np.ones(nj)*ind[2,i])
#     tmp = np.matlib.repmat([indnq,indnqdot,indnqddot],N_tot+1,1).reshape((N_tot+1)*nq*nj,1) > 0
#     if i == 0:
#         indexer = tmp
#     else:
#         indexer = np.concatenate((indexer,tmp),axis=1)

ind = np.eye(nq+1)
for i in range(nq+1):
    for m in range(M):
        indnq = list(np.ones(nj)*ind[0,i])
        indnqdot = list(np.ones(nj)*ind[1,i])
        indnqddot = list(np.ones(nj)*ind[2,i])
        indh = list(np.ones(1)*ind[3,i])
        indnstate = list(np.array([indnq,indnqdot,indnqddot]).reshape(-1,1).flatten())
        if m == 0:
            n_rep = N_stage[m] + 1
        else:
            n_rep = N_stage[m]
        tmp = np.matlib.repmat(indnstate,1,n_rep).flatten()
        # print tmp
        tmp = np.concatenate((np.asarray(indh),tmp)).reshape(-1,1) > 0
        # print tmp
        if m == 0:
            tmp_stage = tmp
        if m > 0:
            tmp_stage = np.concatenate((tmp_stage,tmp),axis=0)
    if i == 0:
        indexer = tmp_stage
    else:
        indexer = np.concatenate((indexer,tmp_stage),axis=1)

# print len(indexer)
w_opt = sol['x'].full()
# print w_opt
# if var_h_opt:
#     h_opt = w_opt[0]
#     w_opt = w_opt[1:]
# else:
#     h_opt = h
w_deb = w_opt[700:]
lbw_deb = lbw[700:]
ubw_deb = ubw[700:]
q_opt = w_opt[indexer[:,0]].reshape(-1,nj)
qdot_opt = w_opt[indexer[:,1]].reshape(-1,nj)
qddot_opt = w_opt[indexer[:,2]].reshape(-1,nj)
h_opt = w_opt[indexer[:,3]]
# print h_opt
rate_opt = int(1/h_opt[0])
# print h_opt
# print np.array(N_stage).reshape(-1,1)
Tf_opt = h_opt*np.array(N_stage).reshape(-1,1)
# print Tf_opt
T_opt = np.zeros([N_tot+1,1])
T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
T_opt[N_cum[0]:N_cum[1]+1] = np.matlib.linspace(Tf_opt[0],Tf_opt[0]+Tf_opt[1],N_stage[1]+1)
# print np.matlib.linspace(Tf_opt[0],Tf_opt[0]+Tf_opt[1],N_stage[1]+1)
# print T_opt
# print T_opt[N_cum[0]+1:N_cum[1]+1]

g_opt = sol['g'].full()

tau_opt = g_opt[tau_ind].reshape(-1,nj)
tau_opt = np.vstack((tau_opt,tau_opt[-1,:]))
# tau_opt[0,:] = DM.nan(1,nj).full()
# print np.shape(tau_opt)
# print g_opt[1400:]
# print tau_ind
# print q_opt[5]

# print q_opt[124]
# print q_opt[125]
# print q_opt[126]


#==============================================================================
#   RETRIEVE CONFIGURATION TRAJECTORY
#==============================================================================

xyz = np.zeros([N_tot+1,3])
for j in range(N_tot+1):
    xyz[j,:] = forKin(q=q_opt[j,:])['ee_pos'].full().reshape(-1,3)

#==============================================================================
#   CREATE SOLVERPARAM & ROBOTPOSE CLASSES, THEN SAVE TO FILE IF DESIRED
#==============================================================================

# CREATE A CLASS FOR THIS WHERE WE STORE Q,QDOT,QDDOT,TAU AND ALL THE OPTIMIZER STUFF
solver_param = fn.SolverParam(solver.stats(),N,h_opt,T,Tf,sol['f'].full())
pose = fn.RobotPose(q=q_opt,qdot=qdot_opt,qddot=qddot_opt,tau=tau_opt,rate=int(1/h_opt[0]))

if var_save:
    solver_param.saveMat()
    pose.saveMat()

#==============================================================================
#   PRINT STATEMENTS
#==============================================================================
print "The initial time step size: h_0 = %s" %h0_vec
print "The initial final time: Tf_0 = %s" %Tf0_vec
print "The optimized time step size: h_opt = %s" %h_opt.flatten().tolist()
print "The optimized final time: Tf_opt = %s" %Tf_opt.flatten().tolist()

impact_ind = N_stage[0]
jacEE_opt = jacEE(q=q_opt[impact_ind,:])['J']
print jacEE_opt
EE_vel_opt = mtimes(jacEE_opt,qdot_opt[impact_ind,:])
B_opt = B_js(q=q_opt[impact_ind,:])['B']
Lambda_ee_opt = mtimes(mtimes(pinv(jacEE_opt).T,B_opt),pinv(jacEE_opt))
h_ee_opt = mtimes(Lambda_ee_opt,EE_vel_opt)
h_ee_zopt = h_ee_opt[2]
h_ee_linopt = h_ee_opt[0:3]
h_ee_resultant = np.linalg.norm(h_ee_linopt)
# EE_vel_opt = np.matmul(jacEE_opt,np.transpose(qdot_opt[-1,:]))
print "The jacobian at the impact: \n %s" %jacEE_opt
print "The optimized final EE linear velocity: %s [m/s]" %EE_vel_opt[0:3]
print "The optimized final EE rotational velocity: %s [rad/s]" %EE_vel_opt[3:]
EE_vel_zopt = EE_vel_opt[2]
print "The optimized final EE velocity along the z-axis: %s [m/s]" %EE_vel_zopt
print "The optimized final joint velocties: %s [rad/s]" %qdot_opt[impact_ind,:]
EE_pos = forKin(q=q_opt[impact_ind,:])['ee_pos']
print "The optimized final momentum h_e: %s [kg m/s]" %h_ee_linopt
print "The optimized final momentum ||h_e||_2: %s [kg m/s]" %h_ee_resultant
print "The optimized final EE cartesian position: %s [m]" %(EE_pos - J01_pos)
print "This position is w.r.t. origin of joint 1"

#==============================================================================
#   PLOTTING THE RESULTS
#==============================================================================

if var_pl or var_save:
    # pose.plot_q(show=var_pl,save=var_save,title=True,block=False,lb=lbq,ub=ubq,limits=True,Tvec=T_opt)
    # pose.plot_qdot(show=var_pl,save=var_save,title=True,block=False,lb=lbqdot,ub=ubqdot,limits=True,Tvec=T_opt)
    # pose.plot_qddot(show=var_pl,save=var_save,title=True,block=False,Tvec=T_opt)
    # tau_lim = np.matlib.repmat(ubtau,pose.N,1)
    ubtau_plt = np.zeros([N_tot+1,nj])
    ubtau_plt[0:N_cum[0]] = np.matlib.repmat(ubtau[:,0].reshape(1,-1),N_cum[0],1)
    ubtau_plt[N_cum[0]:] = np.matlib.repmat(ubtau[:,1].reshape(1,-1),N_stage[1]+1,1)
    # pose.plot_tau(show=var_pl,save=var_save,title=True,block=False,lb=-ubtau_plt,ub=ubtau_plt,limits=True,Tvec=T_opt)
    
    # pose.plot_joint(joint=1,nq=2,show=False,save=False,title=True,block=False)
    # pose.plot_joint(joint=2,nq=2,show=False,save=False,title=True,block=False)

    #PLOT THE EE POSITION OF THE SIMULATION AND THE DESIRED POSITION
    plt.figure()
    c_range = range(N_tot+1)
    var_workspace = True
    if var_workspace:
        # =============================================================================
        #   PARAMETERSS
        # =============================================================================
        #   ANGULAR BOUNDS
        lbq_deg = [-95, -20]
        ubq_deg = [195, 145]
        lbq = np.deg2rad(lbq_deg).tolist()
        ubq = np.deg2rad(ubq_deg).tolist()

        pos_world = EE_pos_N[0:2] # y,z    x is neglected
        pos_world = [0, 1.2] # y,z    x is neglected

        #   VALUES TO CHECK
        y_check = EE_pos_N[1]
        z_check = EE_pos_N[2]

        # y_check += pos_world[0]
        # z_check += pos_world[1]

        # =============================================================================
        #   PLOT REGION OF ADMISSIBLE CONFIGURATIONS
        # =============================================================================
        #               | q_2
        #       A_______|_______________B
        #       |       |               |
        #       |       |               |
        #       |       |               |
        #   ------------------------------------q_1
        #       |       |               |
        #       |       |               |
        #       |       |               |
        #       D_______|_______________C
        #               | 

        N = 200
        Nt = 4*N

        AB = [lbq[0],ubq[0]]        # q_1 changes, q_2 constant
        AB_q1 = np.matlib.linspace(AB[0],AB[1],N+1)
        AB_q2 = np.matlib.repmat(ubq[1],1,N+1).flatten()
        BC = [ubq[1],lbq[1]]        # q_1 constant, q_2 changes
        BC_q1 = np.matlib.repmat(ubq[0],1,N+1).flatten()
        BC_q2 = np.matlib.linspace(BC[0],BC[1],N+1)
        CD = [ubq[0],lbq[0]]        # q_1 changes, q_2 constant
        CD_q1 = np.matlib.linspace(CD[0],CD[1],N+1)
        CD_q2 = np.matlib.repmat(lbq[1],1,N+1).flatten()
        DA = [lbq[1],ubq[1]]        # q_1 constant, q_2 changes
        DA_q1 = np.matlib.repmat(lbq[0],1,N+1).flatten()
        DA_q2 = np.matlib.linspace(DA[0],DA[1],N+1)

        q1 = np.concatenate((AB_q1[:-1],BC_q1[:-1],CD_q1[:-1],DA_q1[:-1]))
        q2 = np.concatenate((AB_q2[:-1],BC_q2[:-1],CD_q2[:-1],DA_q2[:-1]))

        # plt.figure()
        # plt.clf()
        # plt.scatter(q1,q2)
        # plt.show()

        y = np.zeros(Nt+1)
        z = np.zeros(Nt+1)
        dist = np.zeros(Nt+1)

        for i in range(Nt):
            location = forKin(q=[q1[i],q2[i]])['ee_pos']
            y[i] = location[1]
            z[i] = location[2]
            dist[i] = np.linalg.norm([y[i],z[i]-1.2])
        y[-1] = y[0]
        z[-1] = z[0]
        dist[-1] = dist[0]

        sec1 = [0, 3*N]
        sec2 = [3*N, -1]
        max1 = np.amax(dist[sec1[0]:sec1[1]])
        arg1 = np.argmax(dist[sec1[0]:sec1[1]])
        max2 = np.amax(dist[sec2[0]:sec2[1]])
        arg2 = np.argmax(dist[sec2[0]:sec2[1]]) + sec2[0]
        # print arg1
        # print arg2
        N_sec = arg2 - arg1 - 1 # -1 is to compensate for the true inbetween

        ystart = y[arg1+1] - pos_world[0]
        zstart = z[arg1+1] - pos_world[1]
        astart = np.arctan2(zstart,ystart)
        yend = y[arg2-1] - pos_world[0]
        zend = z[arg2-1] - pos_world[1]
        aend = np.arctan2(zend,yend)

        if aend > astart:
            astart += 2*np.pi
        # print astart
        # print aend 
        angle = np.linspace(astart,aend,N_sec)
        r = np.amax([max1, max2])
        y_arc = r * np.cos(angle) + pos_world[0]
        z_arc = r * np.sin(angle) + pos_world[1]
        y2 = np.copy(y)
        z2 = np.copy(z)
        y2[arg1+1:arg2] = y_arc
        z2[arg1+1:arg2] = z_arc


        # plt.figure()
        # plt.clf()
        # plt.fill(y2,z2)
        # plt.scatter(y_check,z_check,s=150,c="r",marker="+",zorder=10)
        # plt.scatter(y[arg2],z[arg2],s=50,c="k",marker="+")
    plt.fill(y,z,"lightblue",alpha=0.2)
    plt.fill(y2,z2,"lightblue",alpha=0.2)
    plt.scatter(xyz[:,1],xyz[:,2],c=c_range,cmap='Greens',edgecolor="k",linewidths=0.5)
    # plt.scatter(xyz[:,1],xyz[:,2],c=c_range,cmap='Greens',marker='+',markeredgewidth=1.5, markeredgecolor='k')
    plt.scatter(xyz[impact_ind,1],xyz[impact_ind,2],s=150,c="r",marker="+")
    plt.grid(True)
    plt.show(block=False)

    EE_vel_T_ts = np.zeros([N_stage[0]+1,nj])
    for i in range(N_stage[0]+1):
        jacEE_Ti = jacEE(q=q_opt[i,:])['J']
        jacEE_Ti_ts = jacEE_Ti[1:3,:] # Task space jacobian
        # print jacEE_Ti_ts
        EE_vel_Ti_ts = mtimes(jacEE_Ti_ts,qdot_opt[i,:]) # Task space velocity vector
        # print EE_vel_Ti_ts
        EE_vel_T_ts[i,:] = EE_vel_Ti_ts.full().flatten()
    # print EE_vel_T_ts
    zero_vec = np.zeros([N_stage[0]+1,1])
    plt.figure()
    plt.quiver(range(N_stage[0]+1),zero_vec,EE_vel_T_ts[:,0],EE_vel_T_ts[:,1])
    plt.show(block=False)


    if var_pl:
        plt.show()
    
#==============================================================================
#   ANIMATING THE RESULTS WITH RVIZ
#   HAS TO HAPPEN AFTER PLOTTING AND SAVING BECAUSE IT OVERWRITES THE REAL POSE
#==============================================================================

if var_ani:
    # pose.interpolate(rviz_rate)
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

# print np.shape(q_opt)