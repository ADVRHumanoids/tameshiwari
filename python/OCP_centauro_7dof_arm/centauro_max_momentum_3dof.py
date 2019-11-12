#!/usr/bin/env python2
# -*- coding: utf-8 -*-

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia

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
import centauro_functions as cfn
import joint_state_centauro as jsc
import joint_state as js
import init_state
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

arm_left = True
arm_both = True
if arm_left:
    j_arm = ['j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7']
    j_arm_lb = ['-3.312', '0.020', '-2.552', '-2.465', '-2.569', '-1.529', '-2.565']
    j_arm_ub = ['1.615', '3.431', '2.566', '0.280', '2.562', '1.509', '2.569']
else:
    j_arm = ['j_arm2_1', 'j_arm2_2', 'j_arm2_3', 'j_arm2_4', 'j_arm2_5', 'j_arm2_6', 'j_arm2_7']
    j_arm_lb = ['-3.3458', '-3.4258', '-2.5614', '-2.4794', '-2.5394', '-1.5154', '-2.5554']
    j_arm_ub = ['1.6012', '-0.0138', '2.5606', '0.2886', '2.5546', '1.5156', '2.5686']
if arm_both:
    j_arm = ['j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7']
    j_arm_lb = ['-3.312', '0.020', '-2.552', '-2.465', '-2.569', '-1.529', '-2.565']
    j_arm_ub = ['1.615', '3.431', '2.566', '0.280', '2.562', '1.509', '2.569']
    j_arm += ['j_arm2_1', 'j_arm2_2', 'j_arm2_3', 'j_arm2_4', 'j_arm2_5', 'j_arm2_6', 'j_arm2_7']
    j_arm_lb += ['-3.3458', '-3.4258', '-2.5614', '-2.4794', '-2.5394', '-1.5154', '-2.5554']
    j_arm_ub += ['1.6012', '-0.0138', '2.5606', '0.2886', '2.5546', '1.5156', '2.5686']

print j_arm
joint_num = [1]
q_name = [j_arm[j] for j in [i for i,x in enumerate(j_arm) if int(x[-1]) in joint_num]]
print q_name
q_lb = [j_arm_lb[j] for j in [i for i,x in enumerate(j_arm) if x in q_name]]
q_lb = [float(i) for i in q_lb]
print q_lb
q_ub = [j_arm_ub[j] for j in [i for i,x in enumerate(j_arm) if x in q_name]]
q_ub = [float(i) for i in q_ub]
print q_ub

N = 500
q = np.empty((N,0))
# if arm_both:
#     joint = [1,3,4,6]
# else:
#     joint = [1]
reverse = False
for i, name in enumerate(q_name):
    # if i+1 in joint:
    #     q_i = np.matlib.linspace(q_lb[i],q_ub[i],N).reshape(-1,1)
    #     if reverse:
    #         q_i = np.flipud(q_i)
    # else:
    #     q_i = np.zeros((N,1))
    q_i = np.matlib.linspace(q_lb[i],q_ub[i],N).reshape(-1,1)
    q = np.append(q, q_i, axis=1)

# print q
pose = fn.RobotPose(name=q_name,q=q,rate=30)
js.posePublisher(pose)