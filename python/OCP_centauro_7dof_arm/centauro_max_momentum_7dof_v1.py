#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#   Extension of v1 where here there is a vector based optimization.
#   This implies that a normal vector to hitting surface must be provided
#   in order to have the optimizer find optimal hitting trajectory.

#   Additionally, a workspace constraint is enforced base on the normal
#   vector of the hitting surface. This constraint represents a 2D box.
#   Perpendicular to this box surface the contraints are limited infinitely.

#   Adding Torso 

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
import math
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D

#==============================================================================
#   CUSTOM PACKAGES
#==============================================================================

import functions as fn
import centauro_functions as cfn
import centauro_config as config
import joint_state_centauro as jsc
import joint_state as js
import init_state_centauro
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import centauro_inverse_kinematics as invKyn

# HITTING SURFACE NORMAL VECTOR

move = 'chop'           # punch or chop


if move == 'chop':
    n_hat = [0, 0, 1]       # For example this vector is normal to horizontal board which must be hit from the top.
    constr_initial_pose = True
    p_constraint = True
    initial_guess = True
    offdirectional_momentum = False
    postimpact_velocity = False
    minimize_torque = True
    limit_workspace = True         # set to False if want to run an optimizer without endpoint
    limit_ball = True
    torso_fixed = True
    minimize_time = False

    Q_weight = 0.01

    z_adjust = -0.113 # [m]
    p_end = [0.9, 0.0, 1.24+z_adjust]           # THIS WAS USED BEFORE AND WAS HITTING THE BOARD BEFORE REAL MOTION STARTED

elif move == 'punch':
    n_hat = [-1, 0 , 0]     # This is for example a vector normal to a board that is mounted vertically.
    constr_initial_pose = True
    p_constraint = True
    initial_guess = True
    offdirectional_momentum = False
    postimpact_velocity = True
    minimize_torque = True
    limit_workspace = True         # set to False if want to run an optimizer without endpoint
    limit_ball = True
    torso_fixed = False
    minimize_time = False

    # Q_weight = 0.000001
    Q_weight = 0.001

    p_end = [0.8, 0.5, 1.35]                    # Nice one for hitting vertical board.

else:
    sys.exit('####### RUNTIME ERROR: Wrong move selected! #######\n')

u_hat = n_hat + [0,0,0]

init_pose = 'home'

# LOAD THE URDF & DEFINE CASADI CLASS

load_file = True
if load_file:
    dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] +  '/casadi_urdf'
    if torso_fixed:
        fileName = "%s/centauro_urdf_6dof_joints_1111110.txt" % dirName
    else:
        fileName = "%s/centauro_urdf_7dof_joints_1111110_torso_test.txt" % dirName
    with open(fileName, 'r') as f:
        urdf = f.read()
        # print urdf
else:
    urdf = rospy.get_param('robot_description')

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

if torso_fixed:
    joints = config.JointNames('arm1')
else:
    joints = config.JointNames('torso')
    joints.addJoints('arm1')
joint_str = joints.getName()

# print joint_str
joint_num = [1,2,3,4,5,6]
# joint_str = [joint_str[j] for j in [i for i,x in enumerate(joint_str) if int(x[-1]) in joint_num]]
for joint in joint_str:
    # print joint
    # print type(joint)
    try:
        if int(joint[-1]) not in joint_num:
            joint_str.remove(joint)
    except:
        pass
# print joint_str
nj = len(joint_str)
nq = 3


prot_str = ['j_arm1_5', 'j_arm1_6']
prot_list = [i for i, joint in enumerate(joint_str) if joint in prot_str]
print prot_list

# print joint_str
# JOINT LIMITS

joint_lim = config.JointBounds(joint_str)
q_lb = joint_lim.getLowerBound()
q_ub = joint_lim.getUpperBound()

# print q_lb 
# print q_ub
# q_lb = [-3.312, 0.04, -2.465]
# q_ub = [1.615, 0.04, 0.28]
qdot_ub = joint_lim.getVelocityBound()
factor = 1.0
# factor = 3.0
qdot_ub = [x*factor for x in qdot_ub]
qdot_lb = [-x for x in qdot_ub]
qddot_ub = 100*np.ones(nj)
qddot_ub = qddot_ub.flatten().tolist()
qddot_lb = [-x for x in qddot_ub]
tau_ub = joint_lim.getTorqueBound() # [143.00 143.00]
tau_lb = [-x for x in tau_ub]
W_tau = [0.6, 0.8]
# W_tau = [0.6, 1.0]
# W_tau = [0.6, 2.0]
# W_tau = [1.6, 2.0]

# TIME LIMITS

N = 120
M = 2           # multi-stage coefficient
N_stage = [N, 60]
N_cum = np.cumsum(N_stage)
iter_max = 0

if constr_initial_pose:
    Tf_1 = 2
else:
    Tf_1 = 1

N_tot = np.sum(N_stage)
data_points = N_tot + 1
# print type(N_tot)
Tf0_list = [Tf_1, 1]
Tf0_vec = np.asfarray(np.array(Tf0_list,ndmin=2))
# Tf_lb = [Tf0_list[0], Tf0_list[1]]
Tf_lb = [Tf0_list[0], 0.05]
Tf_lb = [Tf0_list[0], 0.001]
# Tf_lb = [Tf0_list[0], 0.2]
# Tf_lb = [Tf0_list[0], 0.1]
Tf_lb = np.asfarray(np.array(Tf_lb,ndmin=2))
print np.shape(Tf_lb)
# Tf_ub = [Tf0_list[0], Tf0_list[1]]
Tf_ub = [Tf0_list[0], 3]
Tf_ub = np.asfarray(np.array(Tf_ub,ndmin=2))

if minimize_time:
    Tf_lb[:,0] = Tf_lb[:,1]
    Tf_ub[:,0] = Tf_ub[:,1]
    

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
# p_start = [0.7, 0.16, 1.26]
q_0 = config.HomePose(pose=init_pose,name=joint_str).getValue()
print q_0
# cont, q_0 = invKyn.invKin(p_des=p_start,fk=forKin,frame=end_effector,j_str=joint_str,q_init=q_0,animate=False,T=2)
# q_0 = q_0.tolist()
# q_0 = [0.0,0.0,0.0]
qdot_0 = np.zeros(nj).tolist()
qddot_0 = np.zeros(nj).tolist()

p_0 = forKin(q=q_0)['ee_pos'].full().flatten()
R_0 = forKin(q=q_0)['ee_rot'].full()
p_torso = [0.2, 0.0, 1.19045]
ball_radius = 0.75/2


theta_lb = math.pi/2
theta_ub = math.pi

# p_end = [1.0, 0.0, 1.35] 
# p_end = [1.00696, 0.0, 1.23751]
# p_end = [1.00696, 0.0724726, 1.23751]
# p_end = [0.9, 0.0, 1.24]
# p_end = [0.8, 0.3, 1.24]                    # This is for hitting vertical board.
# p_end = [0.7, 0.4, 1.45]
# p_end = [0.99, 0.0, 1.24]                   # THIS IS VERY VERY CLOSE TO SINGULARITY
# p_end = [0.7, 0.0, 1.0]                     # BOARD ON HIP LEVEL CLOSE TO TORSO
cont, q_end = invKyn.invKin(p_des=p_end,fk=forKin,frame=end_effector,j_str=joint_str,q_init=q_0,animate=False,T=2)

# INITIAL GUESS ON JOINT POSITION
q_0_vec = ml.linspace(q_0,q_end,N_stage[0]+1).transpose()

# cont = True
if not cont:
    sys.exit('####### RUNTIME ERROR: Invalid desired p_end defined! #######\n')
if not fn.inWorkspace(p_0,p_end,n_hat):
    sys.exit('####### RUNTIME ERROR: Initial position is not in workspace! #######\n')

# HOMING
# init_state_centauro.homing(pose=init_pose)

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
#   SEVERAL CONSTRAINT CONCATENATION VECTORS
q_lbopt = np.zeros([nj,N_tot+1])
q_ubopt = np.zeros([nj,N_tot+1])
qdot_lbopt = np.zeros([nj,N_tot+1])
qdot_ubopt = np.zeros([nj,N_tot+1])
qddot_lbopt = np.zeros([nj,N_tot+1])
qddot_ubopt = np.zeros([nj,N_tot+1])

#   CREATE SOME PREDEFINED VARIABLES
gamma_iota = MX(6,1)
gamma_iota[2] = MX([1])
p_iota = MX(p_end)
n_hatMX = MX(3,1)
# n_hatMX = MX(n_hat)
for i,n_i in enumerate(n_hat):
    if n_i != 0:
        n_hatMX[i] = n_i
# USE: sparsify to remove numerical zeros
u_hatMX = vertcat(n_hatMX,MX(3,1))

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
q_lbopt[:,0] = q_lb
q_ubopt[:,0] = q_ub
qdot_lbopt[:,0] = qdot_lb
qdot_ubopt[:,0] = qdot_ub
qddot_lbopt[:,0] = qddot_lb
qddot_ubopt[:,0] = qddot_ub

if p_constraint and initial_guess:
    w0 += q_0_vec[:,0].tolist()
    w0 += qdot_0 + qddot_0
else:
    w0 += q_0 + qdot_0 + qddot_0

#   INITIAL CONDITIONS STATE VECTOR (POSITION, VELOCITY & ACCELERATION)
if constr_initial_pose:
    g += [qk,qdotk]
    lbg += q_0 + qdot_0
    ubg += q_0 + qdot_0
else:
    g += [qdotk]
    lbg += qdot_0
    ubg += qdot_0

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

            #   LIMIT THE WORKSPACE BASED ON THE NORMAL VECTOR TO THE IMPACT SURFACE
            if limit_workspace and p_constraint:
                pk = forKin(q=qk)['ee_pos']
                variance = minus(pk,p_iota)
                inner_prod = dot(n_hatMX,variance)
                g += [inner_prod]
                lbg += [0]
                ubg += [inf]

            if limit_ball:
                pk = forKin(q=qk)['ee_pos']
                p_T = MX(p_torso)
                p_TtoE = minus(pk,p_T)
                g += [norm_2(p_TtoE)]
                lbg += [ball_radius]
                ubg += [inf]
            
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
            q_lbopt[:,k+1] = q_lb
            q_ubopt[:,k+1] = q_ub
            qdot_lbopt[:,k+1] = qdot_lb
            qdot_ubopt[:,k+1] = qdot_ub
            qddot_lbopt[:,k+1] = qddot_lb
            qddot_ubopt[:,k+1] = qddot_ub
            # w0 += q_0 + qdot_0 + qddot_0
            if p_constraint and initial_guess:
                w0 += q_0_vec[:,k+1].tolist()
                w0 += qdot_0 + qddot_0
            else:
                w0 += q_0 + qdot_0 + qddot_0

            if minimize_time:
                L = dt
                V += L
                L = dot(qdotk,qdotk)/dot(qdot_ub,qdot_ub)*0.01
                V += L

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
            ubg += [0.005]          # 5 millimeter deviation allowed
            # ubg += [0.05]          # 5 centimeter deviation allowed
        # else:
        #     pk = forKin(q=qk)['ee_pos']
        #     pdel = minus(pk,MX(p_torso))
        #     projection = dot(MX([0,0,1]),pdel)
        #     g += [projection]
        #     lbg += [0]
        #     ubg += [inf]

        approach_constraint = True
        if approach_constraint:
            Rk = forKin(q=qk)['ee_rot']
            nk_be, sk_be, ak_be = horzsplit(Rk,range(4))
            # nk_be is the EE x-axis wrt base x-axis, sk_be the y-axis, and ak_be the z-axis
            # Ensure that the approach axis ak_be has angle wrt the plane greater than theta_lb
            thetak = acos(dot(n_hatMX,ak_be))
            g += [thetak]
            lbg += [theta_lb]
            ubg += [theta_ub]

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
        mu = 10**(-2)
        mu_scalar = 0
        # mu_scalar = 10**(-2)
        Lamk = inv(mtimes(Jk_ts,mtimes(inv(Bk),Jk_ts.T))+mu*MX.eye(sz))
        # Lamk = inv(mtimes(Jk_ts,mtimes(inv(Bk),Jk_ts.T)))
        hk_ts = mtimes(Lamk,vk_ts)
        if nj == 3:
            hk_x, hk_y, hk_z = vertsplit(hk_ts,range(4))
        else:
            hk_x, hk_y, hk_z, hk_rot = vertsplit(hk_ts,range(4) + [sz])
        # E = -hk_x
        # E = -dot(hk_z,hk_z)*100000
        # E = hk_z
        # E = -1*power(dot(u_hatMX,hk_ts),2)
        # E = dot(u_hatMX,hk_ts)    # WORKS BEST
        # E = dot(n_hatMX,pdotk)
        # E = -dot(hk_z,hk_z)
        # V += E
        experimental = False
        if experimental:
            # Experiment with 
            J_hat = mtimes(u_hatMX.T,Jk)
            Lambda_hat = inv(mtimes(J_hat,mtimes(inv(Bk),J_hat.T))+MX(mu_scalar))
            h_hat = mtimes(Lambda_hat,mtimes(u_hatMX.T,vk))
            E = h_hat
            V += E
            E = dot(pdotk - dot(pdotk,n_hatMX)*n_hatMX,pdotk - dot(pdotk,n_hatMX)*n_hatMX)*0.1
            V += E
        else:
            E = dot(u_hatMX,hk_ts)    # WORKS BEST
            V += E
            if offdirectional_momentum:
                E = dot(hk_ts - dot(hk_ts,u_hatMX)*u_hatMX,hk_ts - dot(hk_ts,u_hatMX)*u_hatMX)
                V += E

        if minimize_torque:
            tau_iota = mtimes(Jk.T,u_hatMX)
            w_tau_iota = 1000
            W_tau_iota = MX(nj,nj)
            for ind in prot_list:
                W_tau_iota[ind,ind] = w_tau_iota
            # W_tau_iota[4,4] = w_tau_iota
            # W_tau_iota[5,5] = w_tau_iota
            print W_tau_iota
            E_2 = mtimes(tau_iota.T,mtimes(W_tau_iota,tau_iota))
            V += E_2
        
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
            q_lbopt[:,k+1] = q_lb
            q_ubopt[:,k+1] = q_ub
            qdot_lbopt[:,k+1] = qdot_lb
            qdot_ubopt[:,k+1] = qdot_ub
            qddot_lbopt[:,k+1] = qddot_lb
            qddot_ubopt[:,k+1] = qddot_ub

            experimental_brake = True
            if experimental_brake:
                # Jk = jacEE(q=qk)['J']
                # # print Jk.sparsity()
                # if nj < 6:
                #     Jk_ts, junk = vertsplit(Jk,[0,nj,6])
                #     sz = nj
                # else: 
                #     Jk_ts = Jk
                #     sz = 6
                # vk = mtimes(Jk,qdotk)
                # pdotk, omegak = vertsplit(vk,[0,3,6])
                # pdotk_x, pdotk_y, pdotk_z = vertsplit(pdotk,range(4))
                # vk_ts = mtimes(Jk_ts,qdotk) # Task space velocity vector
                # L = dot(pdotk_x,pdotk_x) + dot(pdotk_y,pdotk_y)
                # V += L
                # L = dot(qk,qk)
                # L = dot(qdotk,qdotk)
                # L = 0
                # weight = 0.001
                # # weight = 1
                # W_qdot = np.eye(nj)*weight
                # L = mtimes(qdotk.T,mtimes(W_qdot,qdotk))
                # L = 0
                # L = dot(qkmin1-qk,qkmin1-qk)
                # L = dot(qdotk,qdotk)/dot(qdot_ub,qdot_ub)*Q_weight
                # L = dot(qdotk,qdotk)/dot(qdot_ub,qdot_ub)
                L = dot(qdotk,qdotk)/dot(qdot_ub,qdot_ub)
                V += L
                L = dt/dt_ub[Mk]
                V += L
                # V += dt/dot(dt_ub[Mk],dt_ub[Mk])
                # V += dot(dt,dt)/dot(dt_ub[Mk],dt_ub[Mk])
                # pass
            else:
                L = dot(qkmin1-qk,qkmin1-qk)
                V += L

            if postimpact_velocity:
                Jk = jacEE(q=qk)['J']
                if nj < 6:
                    Jk_ts, junk = vertsplit(Jk,[0,nj,6])
                    sz = nj
                else: 
                    Jk_ts = Jk
                    sz = 6
                vk = mtimes(Jk,qdotk)
                pdotk, omegak = vertsplit(vk,[0,3,6])
                pdotk_para = dot(pdotk,n_hatMX)*n_hatMX
                pdotk_perp = minus(pdotk,pdotk_para)
                L = dot(pdotk_perp,pdotk_perp)
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
        #   WITHOUT SLACK
        lbg += np.zeros(nj).tolist()
        ubg += np.zeros(nj).tolist()
        #   TERMINAL COST 
        # E = N_stage[Mk]*dt
        # V += E


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
J_N_lin, J_N_rot = vertsplit(J_N,[0,3,6])

v_N = mtimes(J_N,qdot_opt[impact_ind,:])
if nj < 6:
    J_Nts, junk = vertsplit(J_N,[0,nj,6])
else: 
    J_Nts = J_N
Lambda_iota = inv(mtimes(mtimes(J_Nts,inv(B_N)),J_Nts.T)+mu*DM.eye(sz))
v_Nts = mtimes(J_Nts,qdot_opt[impact_ind,:])
pdot_N, omega_N = vertsplit(v_Nts,[0,3,6]) 
h_iota = mtimes(Lambda_iota,v_Nts)
h_iota_rot = h_iota[3:]
h_iota_lin = h_iota[:3]
h_iota_z = h_iota_lin[-1]
h_iota_resultant = np.linalg.norm(h_iota_lin)

u_hat_vec = DM(u_hat)
J_hat_opt = mtimes(u_hat_vec.T,J_N)
Lambda_hat_opt = inv(mtimes(J_hat_opt,mtimes(inv(B_N),J_hat_opt.T))+DM(mu_scalar))
h_hat_opt = mtimes(Lambda_hat_opt,mtimes(u_hat_vec.T,v_N))
Lambda_3x3 = inv(mtimes(J_N_lin,mtimes(inv(B_N),J_N_lin.T)))
h_lin = mtimes(Lambda_3x3,pdot_N)

print "The task space inertia matrix Lambda(q): \n %s" %Lambda_iota
print "The joint space inertia matrix B(q): \n %s" %B_N
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

print "The optimized final rotational momentum h_iota: %s [kg rad/s]" %h_iota_rot
print "The optimized final liinear momentum h_iota: %s [kg m/s]" %h_iota_lin
print "The optimized final momentum ||h_iota||_2: %s [kg m/s]" %h_iota_resultant
print "The optimized momentum performance index h_hat: %s" %h_hat_opt
print "Compare this with the linear momentum h_lin = Lambda_lin*pdot: %s" %h_lin
print "The J_hat scalar at the moment of impact: %s" %J_hat_opt

print "The optimized joint angles at impact: %s [rad]" %q_opt[impact_ind,:]

u_hatDM = DM(u_hat)
tau_iota_opt = mtimes(J_N.T,u_hatDM)
print "The optimized static torque at impact: %s [Nm]" %tau_iota_opt

plot_trace = False
if plot_trace:
    xyz = np.zeros([N_tot+1,3])
    for j in range(N_tot+1):
        xyz[j,:] = forKin(q=q_opt[j,:])['ee_pos'].full().reshape(-1,3)
    dim_2 = False
    if dim_2:
        # 2D
        plt.figure()
        c_range = range(N_tot+1)
        plt.scatter(xyz[:,0],xyz[:,2],c=c_range,cmap='Greens',edgecolor="k",linewidths=0.5)
        # plt.scatter(xyz[:,1],xyz[:,2],c=c_range,cmap='Greens',marker='+',markeredgewidth=1.5, markeredgecolor='k')
        plt.scatter(xyz[impact_ind,0],xyz[impact_ind,2],s=150,c="r",marker="+")
        plt.grid(True)
        plt.show()
    dim_3 = True
    if dim_3:
        # 3D
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(xyz[:,0],xyz[:,1],xyz[:,2])
        ax.scatter(xyz[impact_ind,0],xyz[impact_ind,1],xyz[impact_ind,2],s=150,c="r",marker="+")
        plt.show()

Tf_opt = dt_opt*np.array(N_stage).reshape(-1,1)
if M == 1:
    T_opt = np.zeros([N_stage[0]+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
else:
    T_opt = np.zeros([N_tot+1,1])
    T_opt[0:N_stage[0]+1] = np.matlib.linspace(0,Tf_opt[0],N_stage[0]+1)
    T_opt[N_cum[0]:N_cum[1]+1] = np.matlib.linspace(Tf_opt[0],Tf_opt[0]+Tf_opt[1],N_stage[1]+1)

solver_param = fn.SolverParam(solver.stats(),N_tot+1,dt_opt,T_opt,Tf_opt,sol['f'].full())
evaluate = True
if evaluate:
    evaluation = fn.RobotEvaluation()
    evaluation.addSolverParam(solver_param.dict)
    evaluation.addBoundedParam('q','joint_position',np.transpose(q_opt),q_lbopt,q_ubopt)
    evaluation.addBoundedParam('qdot','joint_velocity',np.transpose(qdot_opt),qdot_lbopt,qdot_ubopt)
    evaluation.addBoundedParam('qddot','joint_acceleration',np.transpose(qddot_opt),qddot_lbopt,qddot_ubopt)
    tau_ubopt = np.zeros([N_tot+1,nj])
    tau_ubopt[0:N_cum[0]] = np.matlib.repmat(W_tau[0]*np.asarray(tau_ub).reshape(1,-1),N_cum[0],1)
    tau_ubopt[N_cum[0]:] = np.matlib.repmat(W_tau[1]*np.asarray(tau_ub).reshape(1,-1),N_stage[1]+1,1)
    tau_lbopt = -1*tau_ubopt
    evaluation.addBoundedParam('tau','joint_effort',np.transpose(tau_opt),tau_lbopt,tau_ubopt)
    evaluation.addBoundedParam('Tf','final_time',Tf_opt,Tf_lb,Tf_ub)
    evaluation.addParam('time',T_opt)
    evaluation.addParam('joints',joint_str)
    evaluation.addParam('n_hat',n_hat)
    evaluation.addParam('impact_postion',p_end)
    evaluation.addParam('stage_interval',N_stage)
    evaluation.addParam('data_points',data_points)
    evaluation.addParam('mu_regular',mu)
    eval_momentum = True
    if eval_momentum:
        twist_opt = np.zeros([6,data_points])
        jacobian_opt = np.zeros([6,nj,data_points])
        joint_inertia_opt = np.zeros([nj,nj,data_points])
        task_inertia_opt = np.zeros([6,6,data_points])
        momentum_vector_opt = np.zeros([6,data_points])
        momentum_scalar_opt = np.zeros([1,data_points])
        for k in range(data_points):
            q_k = q_opt[k,:]
            qdot_k = qdot_opt[k,:]
            jacobian_k = jacEE(q=q_k)['J']
            twist_k = mtimes(jacobian_k,qdot_k)
            joint_I_k = inertiaJS(q=q_k)['B']
            task_I_k = inv(mtimes(jacobian_k,mtimes(inv(joint_I_k),jacobian_k.T))+mu*DM.eye(6))
            h_vector_k = mtimes(task_I_k,twist_k)
            h_scalar_k = mtimes(u_hatDM.T,h_vector_k)

            twist_opt[:,k] = twist_k.full().flatten()
            jacobian_opt[:,:,k] = jacobian_k.full()
            joint_inertia_opt[:,:,k] = joint_I_k.full()
            task_inertia_opt[:,:,k] = task_I_k.full()
            momentum_vector_opt[:,k] = h_vector_k.full().flatten()
            momentum_scalar_opt[:,k] = h_scalar_k.full()
        

joint_str += ['j_arm1_7']

print joint_str
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

# print np.shape(q_opt)
pose = fn.RobotPose(name=joint_str,q=q_opt,qdot=qdot_opt,qddot=qddot_opt,tau=tau_opt,rate=int(1/dt_opt[0]))
# print joint_str
# print pose.name

plot_joints = True
if plot_joints:
    pose.plot_q(lb=q_lb,ub=q_ub,limits=True,Tvec=T_opt,nj_plot=nj)
    pose.plot_qdot(lb=qdot_lb,ub=qdot_ub,limits=True,Tvec=T_opt,nj_plot=nj)
    pose.plot_qddot(lb=qddot_lb,ub=qddot_ub,limits=True,Tvec=T_opt,nj_plot=nj)
    ubtau_plt = np.zeros([N_tot+1,nj])
    ubtau_plt[0:N_cum[0]] = np.matlib.repmat(W_tau[0]*np.asarray(tau_ub).reshape(1,-1),N_cum[0],1)
    ubtau_plt[N_cum[0]:] = np.matlib.repmat(W_tau[1]*np.asarray(tau_ub).reshape(1,-1),N_stage[1]+1,1)
    lbtau_plt = -1*ubtau_plt
    pose.plot_tau(lb=lbtau_plt,ub=ubtau_plt,limits=True,Tvec=T_opt,nj_plot=nj)

###################################################################################################

run_final = True
if run_final:
    #   CREATE TWO POSE CLASSES: STAGE_1 AND STAGE_2
    #   INTERPOLATE TO A DESIRED SAMPLE RATE
    q_s1 = q_opt[0:N_cum[0]+1,:]
    q_s2 = q_opt[N_cum[0]:,:]
    #   there is one frame overlap, required for the interpolation
    qdot_s1 = qdot_opt[0:N_cum[0]+1,:]
    qdot_s2 = qdot_opt[N_cum[0]:,:]
    qddot_s1 = qddot_opt[0:N_cum[0]+1,:]
    qddot_s2 = qddot_opt[N_cum[0]:,:]
    tau_s1 = tau_opt[0:N_cum[0]+1,:]
    tau_s2 = tau_opt[N_cum[0]:,:]

    rate_s1 = int(1/dt_opt[0])
    rate_s2 = int(1/dt_opt[1])

    pose_s1 = fn.RobotPose(name=joint_str,q=q_s1,qdot=qdot_s1,qddot=qddot_s1,tau=tau_s1,rate=rate_s1)
    pose_s2 = fn.RobotPose(name=joint_str,q=q_s2,qdot=qdot_s2,qddot=qddot_s2,tau=tau_s2,rate=rate_s2)
    pose_s1.interpolate(30)
    pose_s2.interpolate(30)

    #   CONCATENATE THE TWO SEPERATE POSES
    if pose_s1.rate == pose_s2.rate:
        q_s1 = pose_s1.q
        q_s2 = pose_s2.q[1:-1,:]
        qdot_s1 = pose_s1.qdot
        qdot_s2 = pose_s2.qdot[1:-1,:]
        qddot_s1 = pose_s1.qddot
        qddot_s2 = pose_s2.qddot[1:-1,:]
        tau_s1 = pose_s1.tau
        tau_s2 = pose_s2.tau[1:-1,:]
        q_msg = np.concatenate((q_s1,q_s2))
        qdot_msg = np.concatenate((qdot_s1,qdot_s2))
        qddot_msg = np.concatenate((qddot_s1,qddot_s2))
        tau_msg = np.concatenate((tau_s1,tau_s2))
        pose_msg = fn.RobotPose(name=joint_str,q=q_msg,qdot=qdot_msg,qddot=qddot_msg,tau=tau_msg,rate=pose_s1.rate)
        save = False
        if save:
            pose_msg.saveMat()
        raw_input('Press enter to continue, get ready to record: ')
        # jsc.posePublisher(pose=pose_msg,init_pose=init_pose)
        jsc.posePublisher(pose=pose_s1,init_pose=init_pose)
        raw_input('Press enter to continue, get ready to record: ')
        jsc.posePublisher(pose=pose_s2,init_pose=init_pose)
    else:
        print "ERROR, the two stages don't have equal framerate"