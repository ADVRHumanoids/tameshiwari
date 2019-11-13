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
import centauro_config as config
import joint_state_centauro as jsc
import joint_state as js
import init_state
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import centauro_inverse_kinematics as invKyn

joint_str = config.JointNames('arm1').getName()
joint_num = [1,2,4]
joint_str = [joint_str[j] for j in [i for i,x in enumerate(joint_str) if int(x[-1]) in joint_num]]
joint_lim = config.JointBounds(joint_str)
q_lb = joint_lim.getLowerBound()
q_ub = joint_lim.getUpperBound()

load_file = False
if load_file:
    dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] +  '/casadi_urdf'
    fileName = "%s/centauro_2dof_urdf.txt" % dirName
    with open(fileName, 'r') as f:
        urdf = f.read()
        print urdf
else:
    urdf = rospy.get_param('robot_description')

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
end_effector = 'arm1_8'
invDyn = Function.deserialize(kindyn.rnea())
forKin = Function.deserialize(kindyn.fk(end_effector))
jacEE = Function.deserialize(kindyn.jacobian(end_effector))
inertiaJS = Function.deserialize(kindyn.crba())

# INITIAL JOINT STATE
q_0 = config.HomePose(name=joint_str).getValue()

p_end = [1.0, 0.0, 1.35]
q_end = invKyn.invKin(fk=forKin,j_str=joint_str,q_init=q_0,animate=False,T=2)

print type(q_lb)