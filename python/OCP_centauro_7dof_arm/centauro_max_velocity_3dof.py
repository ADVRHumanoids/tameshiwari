#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#==============================================================================
#   RELEASED PACKAGES
#==============================================================================

from casadi import *
import os, sys
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
import centauro_functions as cfn
import centauro_config as config
import joint_state_centauro as jsc
import joint_state as js
import init_state
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import centauro_inverse_kinematics as invKyn

init_pose = 'home'

# LOAD THE URDF & DEFINE CASADI CLASS

load_file = True
if load_file:
    dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] +  '/casadi_urdf'
    fileName = "%s/centauro_urdf_3dof_joints_1101000.txt" % dirName
    # fileName = "%s/centauro_urdf_6dof_joints_1111110.txt" % dirName
    with open(fileName, 'r') as f:
        urdf = f.read()
        # print urdf
else:
    urdf = rospy.get_param('robot_description')

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

joint_str = config.JointNames('arm1').getName()
print joint_str
joint_num = [1,2,4]
# joint_num = [1,2,3,4,5,6]
joint_str = [joint_str[j] for j in [i for i,x in enumerate(joint_str) if int(x[-1]) in joint_num]]
nj = len(joint_str)
nq = 3

print joint_str
# JOINT LIMITS

joint_lim = config.JointBounds(joint_str)
q_lb = joint_lim.getLowerBound()
q_ub = joint_lim.getUpperBound()
# print q_lb 
# print q_ub
# q_lb = [-3.312, 0.04, -2.465]
# q_ub = [1.615, 0.04, 0.28]
qdot_ub = joint_lim.getVelocityBound()
qdot_lb = [-x for x in qdot_ub]
# nj_ones = 
qddot_ub = 100*np.ones(nj)
qddot_ub = qddot_ub.flatten().tolist()
qddot_lb = [-x for x in qddot_ub]
tau_ub = joint_lim.getTorqueBound()
tau_lb = [-x for x in tau_ub]
W_tau = [0.2, 0.6]

# TIME LIMITS

N = 120
M = 2           # multi-stage coefficient
N_stage = [N, 60]
N_cum = np.cumsum(N_stage)
iter_max = 0

N_tot = np.sum(N_stage)
# print type(N_tot)
Tf0_list = [3, 1]
Tf0_vec = np.asfarray(np.array(Tf0_list,ndmin=2))
# Tf_lb = [Tf0_list[0], Tf0_list[1]]
Tf_lb = [Tf0_list[0], 0.05]
Tf_lb = [Tf0_list[0], 0.001]
# Tf_lb = [Tf0_list[0], 0.1]
Tf_lb = np.asfarray(np.array(Tf_lb,ndmin=2))
# Tf_ub = [Tf0_list[0], Tf0_list[1]]
Tf_ub = [Tf0_list[0], 3]
Tf_ub = np.asfarray(np.array(Tf_ub,ndmin=2))

dt_0_vec = (Tf0_vec/N_stage).flatten().tolist()
# print h0_vec
dt_lb = (Tf_lb/N_stage).flatten().tolist()
# print lbh
dt_ub = (Tf_ub/N_stage).flatten().tolist()
# print ubh



# DEFINE CASADI FUNCTION & END-EFFECTOR

end_effector = 'arm1_8'
invDyn = Function.deserialize(kindyn.rnea())
forKin = Function.deserialize(kindyn.fk(end_effector))
jacEE = Function.deserialize(kindyn.jacobian(end_effector))
inertiaJS = Function.deserialize(kindyn.crba())

# print forKin
# print inertiaJS

# INITIAL JOINT STATE
q_0 = config.HomePose(pose=init_pose,name=joint_str).getValue()
# q_0 = [0.0,0.0,0.0]
qdot_0 = np.zeros(nj).tolist()
qddot_0 = np.zeros(nj).tolist()


# p_end = [1.0, 0.0, 1.35] 
# p_end = [1.00696, 0.0, 1.23751]
p_end = [1.00696, 0.0724726, 1.23751]
# p_end = [0.9, 0.0, 1.24]
# p_end = [0.95, 0.0, 1.24]
cont, q_end = invKyn.invKin(p_des=p_end,fk=forKin,frame=end_effector,j_str=joint_str,q_init=q_0,animate=False,T=2)

# INITIAL GUESS ON JOINT POSITION
p_constraint = False
initial_guess = False
q_0_vec = ml.linspace(q_0,q_end,N_stage[0]+1).transpose()

# cont = False
if not cont:
    sys.exit('####### RUNTIME ERROR: Invalid desired p_end defined! #######\n')

# HOMING



# =============================================================================
#   NONLINEAR PROGRAM --> FIND A SOLUTION FOR THE OPTIMAL CONTROL INPUT
# =============================================================================
#   OPTIMIZATION VARIABLES
w = []
w0 = []
lbw = []
ubw = []
#   COST FUNCTION INITIALIZATION
V = 0
#   INEQUALITY CONSTRAINT
g = []
lbg = []
ubg = []
#   TORQUE INDEX VECTOR
tau_ind = []

#   SECTION FOR IF TIME HAS TO BE OPTIMIZED
dt = MX.sym('dt_stage_1')
w += [dt]
w0 += [dt_0_vec[0]]
lbw += [dt_lb[0]]
ubw += [dt_ub[0]]

qk = MX.sym('q_0',nj)
qdotk = MX.sym('qdot_0',nj)
qddotk = MX.sym('qddot_0',nj)
w += [qk,qdotk,qddotk]
lbw += q_lb + qdot_lb + qddot_lb
ubw += q_ub + qdot_ub + qddot_ub
if p_constraint and initial_guess:
    w0 += q_0_vec[:,0].tolist()
    w0 += qdot_0 + qddot_0
else:
    w0 += q_0 + qdot_0 + qddot_0

#   INITIAL CONDITIONS STATE VECTOR (POSITION, VELOCITY & ACCELERATION)
g += [qk,qdotk]
lbg += q_0 + qdot_0
ubg += q_0 + qdot_0

for Mk in range(M):
    if Mk == 0:
        for k in range(N_stage[Mk]):
            #   NEW NLP VARIABLE FOR THE CONTROL
            tauk = invDyn(q=qk,v=qdotk,a=qddotk)['tau']
            for joint in range(nj):
                tau_ind += [len(lbg)+joint]
            g += [tauk]
            lbg += [W_tau[Mk]*tau_lb_j for tau_lb_j in tau_lb]
            ubg += [W_tau[Mk]*tau_ub_j for tau_ub_j in tau_ub]

            L = 0
            V += L

            #   INTEGRATE EULER METHOD
            qdotk_next = qdotk + dt*qddotk 
            qk_next = qk + dt*qdotk + 0.5*qddotk*dt**2

            #   NEW NLP VARIABLE FOR THE CONTROL
            qk = MX.sym('q_' + str(k+1),nj)
            qdotk = MX.sym('qdot_' + str(k+1),nj)
            qddotk = MX.sym('qddot_' + str(k+1),nj)
            w += [qk,qdotk,qddotk]
            lbw += q_lb + qdot_lb + qddot_lb
            ubw += q_ub + qdot_ub + qddot_ub
            # w0 += q_0 + qdot_0 + qddot_0
            if p_constraint and initial_guess:
                w0 += q_0_vec[:,k+1].tolist()
                w0 += qdot_0 + qddot_0
            else:
                w0 += q_0 + qdot_0 + qddot_0

            #   ADD CONTINUITY CONSTRAINTS (ON POSITION AND VELOCITY)
            g += [qk_next - qk,qdotk_next - qdotk]
            lbg += np.zeros(nj*2).tolist()
            ubg += np.zeros(nj*2).tolist()
        
        #   TERMINAL CONSTRAINTS:
        #   TERMINAL POSITION CONSTRAINT
        if p_constraint:
            pk = forKin(q=qk)['ee_pos']
            pk_x, pk_y, pk_z = vertsplit(pk,range(4))
            p_delx = pk_x - p_end[0]
            p_dely = pk_y - p_end[1]
            p_delz = pk_z - p_end[2]
            p_del = vertcat(p_delx,p_dely,p_delz)
            e_norm = norm_2(p_del)
            g += [e_norm]
            lbg += [0.]
            ubg += [0.005]          # 1 millimeter deviation allowed
        #   TERMINAL VELOCITY CONSTRAINT
        #   CONSTRAIN THE Z VELOCITY SUCH THAT THE EE IS GOING TO ARRIVE FROM THE TOP
        # ts_index = DM(np.eye(nj,6))
        Jk = jacEE(q=qk)['J']
        # print Jk.sparsity()
        if nj < 6:
            Jk_ts, junk = vertsplit(Jk,[0,nj,6])
            sz = nj
        else: 
            Jk_ts = Jk
            sz = 6
        vk = mtimes(Jk,qdotk)
        pdotk, omegak = vertsplit(vk,[0,3,6])
        pdotk_x, pdotk_y, pdotk_z = vertsplit(pdotk,range(4))
        vk_ts = mtimes(Jk_ts,qdotk) # Task space velocity vector

        pdot_z_constraint = False
        if pdot_z_constraint:
            g += [pdotk_z]
            lbg += [-inf]
            ubg += [0]

        #   TERMINAL COST --> MOMENTUM MAXIMIZATION
        # E = pdotk_z
        # E = -1*dot(pdotk_z,pdotk_z)
        # V += E
        Bk = inertiaJS(q=qk)['B']
        mu = 5*10**(-2)
        Lamk = inv(mtimes(Jk_ts,mtimes(inv(Bk),Jk_ts.T))+mu*MX.eye(sz))
        # Lamk = inv(mtimes(Jk_ts,mtimes(inv(Bk),Jk_ts.T)))
        hk_ts = mtimes(Lamk,vk_ts)
        if nj == 3:
            hk_x, hk_y, hk_z = vertsplit(hk_ts,range(4))
        else:
            hk_x, hk_y, hk_z, hk_rot = vertsplit(hk_ts,range(4) + [sz])
        # E = -hk_x
        # E = -dot(hk_z,hk_z)*100000
        E = hk_z
        # E = -dot(hk_z,hk_z)
        V += E
        q_N1 = qk
    else:
        dt = MX.sym('dt_stage_2')
        w += [dt]
        w0 += [dt_0_vec[Mk]]
        lbw += [dt_lb[Mk]]
        ubw += [dt_ub[Mk]]

        for k in range(N_stage[Mk-1],N_stage[Mk]+N_stage[Mk-1]):
            #   NEW NLP VARIABLE FOR THE CONTROL
            tauk = invDyn(q=qk,v=qdotk,a=qddotk)['tau']
            for joint in range(nj):
                tau_ind += [len(lbg)+joint]
            g += [tauk]
            lbg += [W_tau[Mk]*tau_lb_j for tau_lb_j in tau_lb]
            ubg += [W_tau[Mk]*tau_ub_j for tau_ub_j in tau_ub]

            #   INTEGRAL COST CONTRIBUTION
            # L = dt*dot(qdotk,qdotk)*0.005
            # L += 0.000001*dt**2
            # L = dt
            # weight = 0.001
            # W_qdot = np.eye(nj)*weight
            # L = mtimes(qdotk.T,mtimes(W_qdot,qdotk))
            # L = weight*dot(qdotk,qdotk)
            # L = mtimes(qdotk.T,qdotk)/dt
            # L = dt
            # V += L

            #   INTEGRATE EULER METHOD
            qdotk_next = qdotk + dt*qddotk 
            qk_next = qk + dt*qdotk  + 0.5*qddotk*dt**2

            qkmin1 = qk

            #   NEW NLP VARIABLE FOR THE CONTROL
            qk = MX.sym('q_' + str(k+1),nj)
            qdotk = MX.sym('qdot_' + str(k+1),nj)
            qddotk = MX.sym('qddot_' + str(k+1),nj)
            w += [qk,qdotk,qddotk]
            lbw += q_lb + qdot_lb + qddot_lb
            ubw += q_ub + qdot_ub + qddot_ub
            w0 += q_0 + qdot_0 + qddot_0

            L = dot(qkmin1-qk,qkmin1-qk)
            # L = dot(qk,qk)
            # L = dot(qdotk,qdotk)
            # L = 0
            # weight = 0.001
            # # weight = 1
            # W_qdot = np.eye(nj)*weight
            # L = mtimes(qdotk.T,mtimes(W_qdot,qdotk))
            # L = 0
            V += L

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
        lbg += np.zeros(nj).tolist()
        ubg += np.zeros(nj).tolist()
        #   TERMINAL COST 
        #   NO TERMINAL COST FUNCTION IS DESIRED
        # E = dot(q_N1-qk,q_N1-qk)
        E = N_stage[Mk]*dt
        V += E


# =============================================================================
#   SOLVER CREATOR AND SOLUTION
# =============================================================================

nlp = dict(f=V, x=vertcat(*w), g=vertcat(*g))
opts = {}
if iter_max > 0:
    opts["ipopt"] = {"max_iter":iter_max}
solver = nlpsol('solver','ipopt',nlp,opts)
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

# =============================================================================
#   RETRIEVE THE OPTIMAL SOLUTIONS
# =============================================================================
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

w_opt = sol['x'].full()
q_opt = w_opt[indexer[:,0]].reshape(-1,nj)
qdot_opt = w_opt[indexer[:,1]].reshape(-1,nj)
qddot_opt = w_opt[indexer[:,2]].reshape(-1,nj)
dt_opt = w_opt[indexer[:,3]]
rate_opt = int(1/dt_opt[0])
Tf_opt = dt_opt*np.array(N_stage).reshape(-1,1)
if M == 1:
    T_opt = np.zeros([N_stage[0]+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
else:
    T_opt = np.zeros([N_tot+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
    T_opt[N_cum[0]:N_cum[1]+1] = np.matlib.linspace(Tf_opt[0],Tf_opt[0]+Tf_opt[1],N_stage[1]+1)

g_opt = sol['g'].full()

tau_opt = g_opt[tau_ind].reshape(-1,nj)
tau_opt = np.vstack((tau_opt,tau_opt[-1,:]))

impact_ind = N_stage[0]
B_N = inertiaJS(q=q_opt[impact_ind,:])['B']
J_N = jacEE(q=q_opt[impact_ind,:])['J']

v_N = mtimes(J_N,qdot_opt[impact_ind,:])
mu = 5*10**(-2)
# J_Nts, junk = J_N[:3,:]
if nj < 6:
    J_Nts, junk = vertsplit(J_N,[0,nj,6])
else: 
    J_Nts = J_N
# J_Nts, junk = vertsplit(J_N,[0,nj,6])
Lambda_iota = inv(mtimes(mtimes(J_Nts,inv(B_N)),J_Nts.T)+mu*DM.eye(sz))
# Lambda_iota = inv(mtimes(mtimes(J_Nts,inv(B_N)),J_Nts.T))
# print type(Lambda_iota)
# print DM.eye(3)
v_Nts = mtimes(J_Nts,qdot_opt[impact_ind,:])
h_iota = mtimes(Lambda_iota,v_Nts)
h_iota_lin = h_iota[:3]
h_iota_z = h_iota_lin[-1]
h_iota_resultant = np.linalg.norm(h_iota_lin)

print "The jacobian at the impact: \n %s" %J_N

print "The initial time step size: h_0 = %s" %dt_0_vec
print "The initial final time: Tf_0 = %s" %Tf0_vec.flatten().tolist()
print "The optimized time step size: h_opt = %s" %dt_opt.flatten().tolist()
print "The optimized final time: Tf_opt = %s" %Tf_opt.flatten().tolist()

print "The optimized final EE linear velocity: %s [m/s]" %v_N[0:3]
print "The optimized final EE rotational velocity: %s [rad/s]" %v_N[3:]
v_Nz = v_N[2]
print "The optimized final EE velocity along the z-axis: %s [m/s]" %v_Nz
print "The optimized final joint velocties: %s [rad/s]" %qdot_opt[impact_ind,:]
p_N = forKin(q=q_opt[impact_ind,:])['ee_pos']
print "The optimized final EE cartesian position: %s [m]" %p_N
print "This position is w.r.t. origin of the world frame"

print "The optimized final momentum h_iota: %s [kg m/s]" %h_iota_lin
print "The optimized final momentum ||h_iota||_2: %s [kg m/s]" %h_iota_resultant

joint_str += ['j_arm1_3','j_arm1_5','j_arm1_6','j_arm1_7']
# joint_str += ['j_arm1_7']
# print type(len(joint_str-nj))
if M == 1:
    zerovec = np.zeros([N_stage[0]+1,len(joint_str)-nj])
else:
    zerovec = np.zeros([N_tot+1,len(joint_str)-nj])
q_opt = np.concatenate((q_opt,zerovec),axis=1)
qdot_opt = np.concatenate((qdot_opt,zerovec),axis=1)
qddot_opt = np.concatenate((qddot_opt,zerovec),axis=1)
tau_opt = np.concatenate((tau_opt,zerovec),axis=1)
# print np.shape(q_opt)

slomo = 1       # if slomo < 1.0 than the motion is slowed down
# print np.shape(q_opt)
pose = fn.RobotPose(name=joint_str,q=q_opt,qdot=qdot_opt,qddot=qddot_opt,tau=tau_opt,rate=int(slomo*1/dt_opt[0]))
# print joint_str
# print pose.name


Tf_opt = dt_opt*np.array(N_stage).reshape(-1,1)
if M == 1:
    T_opt = np.zeros([N_stage[0]+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
else:
    T_opt = np.zeros([N_tot+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
    T_opt[N_cum[0]:N_cum[1]+1] = np.matlib.linspace(Tf_opt[0],Tf_opt[0]+Tf_opt[1],N_stage[1]+1)

pose.plot_q(lb=q_lb,ub=q_ub,limits=True,Tvec=T_opt,nj_plot=nj)
pose.plot_qdot(lb=qdot_lb,ub=qdot_ub,limits=True,Tvec=T_opt,nj_plot=nj)
# pose.plot_qddot(lb=qddot_lb,ub=qddot_ub,limits=True,Tvec=T_opt,nj_plot=nj)
# ubtau_plt = np.zeros([N_tot+1,nj])
# ubtau_plt[0:N_cum[0]] = np.matlib.repmat(ubtau[:,0].reshape(1,-1),N_cum[0],1)
# ubtau_plt[N_cum[0]:] = np.matlib.repmat(ubtau[:,1].reshape(1,-1),N_stage[1]+1,1)

pose.interpolate(30)
# save = False
# if save:
#     pose.saveMat()
# # print pose.rate

jsc.posePublisher(pose=pose,init_pose=init_pose)

