#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

#==============================================================================
#   SUMMARY:
#   This script visualized the planar workspace of the RR-robot
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
#   FORWARD KINEMATICS
# =============================================================================

urdf = rospy.get_param('robot_description')
fk_string = pyn.generate_forward_kin(urdf,'EE')
forKin = Function.deserialize(fk_string)

# =============================================================================
#   PARAMETERSS
# =============================================================================
#   ANGULAR BOUNDS
lbq_deg = [-95, -20]
ubq_deg = [195, 145]
lbq = np.deg2rad(lbq_deg).tolist()
ubq = np.deg2rad(ubq_deg).tolist()

pos_world = [0, 1.2] # y,z    x is neglected

#   VALUES TO CHECK
y_check = 0.65
z_check = 0

y_check += pos_world[0]
z_check += pos_world[1]

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


plt.figure()
plt.clf()
plt.fill(y,z,"lightblue",y2,z2,"lightblue")
# plt.fill(y2,z2)
plt.scatter(y_check,z_check,s=150,c="r",marker="+",zorder=10)
# plt.scatter(y[arg2],z[arg2],s=50,c="k",marker="+")
plt.grid(True)
plt.show(block=False)
# plt.figure()
# plt.plot(range(Nt+1),dist)
# plt.show(block=False)
plt.show()



# zmax = np.amax(z)
# print zmax-1.2

# ymax = np.amax(y)
# print ymax

# print forKin(q=[2.58675695,0.65070416])['ee_pos']
# print forKin()['ee_pos']