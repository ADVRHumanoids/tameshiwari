# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#==============================================================================
#   SUMMARY:
#   This script is an replacement of the rr_robot_min_torque.py where the
#   objective is to minimize over time. That means the time-step
#   of the solver is an optimization variable. Just a single extra variable
#   meaning that it optimizes the duration of a single interval which duration
#   is constant over all N-intervals.
# 
#   OBJECTIVE:
#   The objective is to move the pendulum from the rest (hanging) position
#   to the upward position with the least amount of torque used.
#   The optimization objective is then minimize sum(h*sqrt(dot(tau_k^T,tau_k))).
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

#==============================================================================
#   CUSTOM PACKAGES
#==============================================================================

import functions as fn
import joint_state as js
import tameshiwari.pynocchio_casadi as pyn

# =============================================================================
#   PARAMETERS
# =============================================================================
#   GENERAL PARAMETERS
var_pl      = 1
var_ani     = 1
var_rec     = 0
var_inp     = 0
var_save    = 0
name = 'Res_DMS_minimize_torque'
filename = "%s/%s.mat" % (os.getcwd(),name)

#   SIMULATION PARAMETERS
N = 120
h_0 = 0.02
Tf = N*h_0
T = np.arange(0,Tf+h_0,h_0)
T =  T.reshape(-1,1)
iter_max = 0
rviz_rate = 30

#   INITIAL CONDITIONS (NUMERICAL)
nj = 2
nq = 3 # number of different q's --> q(t), qdot(t), qddot(t)
q_0 = np.zeros(nj).tolist()
# q_0_vec = np.matlib.linspace(0,np.pi,N+1).reshape(-1,1)
# q_0_vec = np.hstack((q_0_vec,np.zeros([N+1,1]))) #.reshape(-1,1).flatten().tolist()
q_0_vec = np.zeros([N+1,nj])
qdot_0 = np.zeros(nj).tolist()
qddot_0 = np.zeros(nj).tolist()
# qddot_0 = np.ones(nj,dtype=float)* 0.5
# qddot_0 = qddot_0.tolist()

#   TERMINAL CONDITIONS (NUMERICAL)
offsetX = 0.0800682
offsetY = 0
offsetZ = 1.2
desX = 0
desY = 0
desZ = 0.797
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
W = 0.5
if var_inp != 0:
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
#   NONLINEAR PROGRAM --> FIND A SOLUTION FOR THE OPTIMAL CONTROL INPUT
# =============================================================================
#   OPTIMIZATION VARIABLES
w = []
w0 = []
lbw = []
ubw = []
h = MX.sym('h')
w += [h]
w0 += [h_0]
lbw = [0.001]
ubw = [0.5]
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

qk = MX.sym('q_0',nj)
qdotk = MX.sym('qdot_0',nj)
qddotk = MX.sym('qddot_0',nj)
w += [qk,qdotk,qddotk]
lbw += lbq + lbqdot + lbqddot
ubw += ubq + ubqdot + ubqddot
# w0 += q_0 + qdot_0 + qddot_0
w0 += q_0_vec[0,:].tolist() + qdot_0 + qddot_0
g += [qk,qdotk,qddotk]
lbg += np.zeros(nj*nq).tolist()
ubg += np.zeros(nj*nq).tolist()

for k in range(N):
    #   NEW NLP VARIABLE FOR THE CONTROL
    tauk = invDyn(q=qk,qdot=qdotk,qddot=qddotk)['tau']
    tau_ind += [np.shape(g)[0]*2,np.shape(g)[0]*2+1]
    g += [tauk]
    lbg += lbtau
    ubg += ubtau

    #   INTEGRAL COST CONTRIBUTION
    # L = dot(tauk,tauk) + dot(qk,qk)
    # L = dot(tauk,tauk)
    L = h
    # L = norm_2(tauk)
    # L = h*sqrt(dot(tauk,tauk))
    # L = dot(qk,qk)
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

#   TERMINAL COST 
# E = dot(qk,qk)
E = 0
J += E

#   TERMINAL CONSTRAINTS:
#   TERMINAL POSITION CONSTRAINT 
EE_pos_k = forKin(q=qk)['ee_pos']
EE_diff_yk = EE_pos_k[1] - EE_pos_N[1]
EE_diff_zk = EE_pos_k[2] - EE_pos_N[2]
EE_diff_k = vertcat(EE_diff_yk,EE_diff_zk)
EE_dist_k = norm_2(EE_diff_k)
g += [EE_dist_k]
lbg += [0.]
ubg += [0.001]          # 1 millimeter deviation allowed
#   TERMINAL VELOCITY CONSTRAINT
jacEE_k = jacEE(q=qk)['J']
EE_vel_k = mtimes(jacEE_k,qdotk)
g += [EE_vel_k]
lbg += np.zeros(6).tolist()
ubg += np.zeros(6).tolist()

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
h_opt = w_opt[0]
w_opt = w_opt[1:]
q_opt = w_opt[indexer[:,0]].reshape(-1,nj)
qdot_opt = w_opt[indexer[:,1]].reshape(-1,nj)
qddot_opt = w_opt[indexer[:,2]].reshape(-1,nj)
# h_opt = h_0

g_opt = sol['g'].full()
tau_opt = g_opt[tau_ind].reshape(-1,nj)

EE_pos = forKin(q=q_opt[-1,:])['ee_pos']
print EE_pos

#==============================================================================
#   RETRIEVE CONFIGURATION TRAJECTORY
#==============================================================================

xyz = np.zeros([N+1,3])
for j in range(N+1):
    xyz[j,:] = forKin(q=q_opt[j,:])['ee_pos'].full().reshape(-1,3)

#==============================================================================
#   Saving variables to file
#==============================================================================

# CREATE A CLASS FOR THIS WHERE WE STORE Q,QDOT,QDDOT,TAU AND ALL THE OPTIMIZER STUFF

#==============================================================================
#   PLOTTING THE RESULTS
#   Plot 1 shows the state trajectories
#   Plot 2 show torque values
#   Plot 3 shows the trajectory in the yz-plane
#==============================================================================

if var_pl != 0:
    plt.figure(1)
    plt.subplot(2,2,1)
    # plt.clf()
    plt.plot(T,q_opt)
    plt.plot(T,qdot_opt)
    plt.legend(('q_J00','q_J01','qdot_J00','qdot_J01'))
    plt.show(block=False)
    # plt.show()

    
    # plt.figure(2)
    plt.subplot(2,2,2)
    # plt.clf()
    plt.step(T,np.vstack((DM.nan(1,nj).full(),tau_opt)))
    tau_lim = np.matlib.repmat(ubtau,N+1,1)
    plt.plot(T,tau_lim)
    plt.plot(T,-tau_lim)
    plt.legend(('tau_J00','tau_J01'))
    plt.show(block=False)    
    # plt.show()

    # plt.figure(3)
    plt.subplot(2,2,3)
    # plt.clf()
    # plt.plot(xyz[:,0],xyz[:,1])
    plt.scatter(xyz[:,1],xyz[:,2])
    # plt.show()
    plt.show(block=False)

    # plt.figure(4)
    plt.subplot(2,2,4)
    # plt.clf()
    fval = np.zeros([N+1])
    for k in range(1,N+1):
        # fval[k] = dot(tau_opt[k-1,:],tau_opt[k-1,:])
        fval[k] = h_opt*mtimes(tau_opt[k-1,:].reshape(1,-1),tau_opt[k-1,:])
    plt.plot(T,fval)
    plt.legend(('fval'))
    plt.show(block=False)

#==============================================================================
#   ANIMATING THE RESULTS WITH RVIZ
#==============================================================================

if var_ani !=0:
    nj      = 2
    j_name  = []
    for j in range(nj):
        tmp     = "J%02d" %(j+1)
        j_name.append(tmp)

    tau_opt = np.vstack((tau_opt,tau_opt[-1,:]))
    # print np.shape(q_opt)
    # print np.shape(qdot_opt)
    # print np.shape(qddot_opt)
    # print np.shape(tau_opt)

    
    pose = fn.RobotPose(j_name,q_opt,qdot_opt,tau_opt,int(1/h_opt))
    # pose.interpolate(rviz_rate)
    if var_rec !=0:
        # os.system("ffmpeg -f x11grab -s 1900x1080 -r 25 -i :0.0 -qscale 5 ~/screenGrab.mpeg")
        # subprocess.call(['ffmpeg','-f','x11grab','-s','1900x1080','r','25','i',':0.0','-qscale','5','screenGrab.mpeg'])
        # subprocess.call("ffmpeg -f x11grab -s 1900x1080 -r 25 -i :0.0 -qscale 5 ~/screenGrab.mpeg", shell=True)
        # subproc = subprocess.Popen("ffmpeg -f x11grab -s 1900x1080 -r 25 -i :0.0 -qscale 5 screenGrab.mpeg", shell=True)
        js.posePublisher(pose)
        # subproc.kill()
        # os.system("q")
    else:
        js.posePublisher(pose)  
    

#==============================================================================
#   DEBUGGING AREA
#==============================================================================




#==============================================================================
#   KEEPING PLOTS OPEN
#==============================================================================
plt.show()