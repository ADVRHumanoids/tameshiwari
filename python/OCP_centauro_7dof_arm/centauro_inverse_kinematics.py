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

def invKin(fk=None,frame=None,j_str=None,q_init=None,p_init=None,p_des=None,T=None,animate=True):
    if not fk:
        print "forward kinematics function not provided"
    else:
        forKin = fk

    frame = frame or 'arm1_8'
    j_str = j_str or config.JointNames('arm1').getName()

    j_lim = config.JointBounds(j_str)
    q_lb = j_lim.getLowerBound()
    q_ub = j_lim.getUpperBound()

    T = T or 1
    nq = forKin.numel_in()
    q = SX.sym('q',nq)

    if q_init is not None:
        p_init = forKin(q=q_init)['ee_pos'].full()
    elif p_init is None and q_init is None:
        print "ERROR: no initial q or p provided"

    p_des = p_des or [1.0, 0.0, 1.35]
    if isinstance(p_des,list):
        p_des = np.array(p_des)
    elif not isinstance(p_des,np.ndarray):
        print "ERROR: in p_des creation"

    p_des_sym = forKin(q=q)['ee_pos']
    p_des_num = SX(p_des)

    J = dot(p_des_sym-p_des_num,p_des_sym-p_des_num)

    nlp = dict(f=J, x=q)
    opts = {"print_time":False}
    opts["ipopt"] = {"print_level":0}
    solver = nlpsol('solver','ipopt',nlp,opts)
    sol = solver(x0=q_init, lbx=q_lb, ubx=q_ub)
    q_sol = sol['x'].full().transpose()
    q_motion = np.append(np.array(q_init),q_sol.flatten()).reshape(2,-1)
    q_dot = np.zeros([2,nq])

    print "######## RUNNING INVERSE KINEMATICS CHECK ########\n"
    print "Final joint configuration: %s [rad]" %q_sol.flatten()
    print "The desired xyz end-effector position: %s" %p_des.tolist()
    p_sol = np.around(forKin(q=q_sol)['ee_pos'].full().flatten(),2)
    print "The achieved xyz end-effector position: %s" %p_sol.tolist()
    e_des = sqrt(sol['f'].full().flatten())
    print "The achieved ||des-res||_2 error: %s [mm]\n" %round(e_des*1000,0)
    tolerance = 0.005 # 5 millimeter
    tolerance = 0.05 # 5 centimeter
    if solver.stats()['return_status'] == 'Solve_Succeeded' and e_des <= tolerance:
        print "######## INVERSE KINEMATICS SUCCEEDED ########\n"
        cont = True
    else:
        print "######## ERROR ######## \n\nDesired end point is not in task space \n\n######## ERROR ########\n"
        cont = False
    
    if animate:
        pose = fn.RobotPose(name=j_str,q=q_motion,qdot=q_dot,rate=1/float(T))
        pose.interpolate(30.0)
        js.posePublisher(pose)

    return cont, q_sol.flatten()



if __name__ == "__main__":
    load_file = True
    if load_file:
        dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] +  '/casadi_urdf'
        fileName = "%s/centauro_urdf_6dof_joints_1111110.txt" % dirName
        with open(fileName, 'r') as f:
            urdf = f.read()
            # print urdf
    else:
        urdf = rospy.get_param('robot_description')
    kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
    end_effector = 'arm1_8'
    forKin = Function.deserialize(kindyn.fk(end_effector))

    joint_str = config.JointNames('arm1').getName()
    joint_num = [1,2,3,4,5,6]
    joint_str = [joint_str[j] for j in [i for i,x in enumerate(joint_str) if int(x[-1]) in joint_num]]
    q_init = config.HomePose(name=joint_str).getValue()
    # print q_init

    p_end = [1.0, 0.0, 1.35]
    p_end = [0.7, 0.16, 1.26]

    init_pose = 'home'
    q_0 = config.HomePose(pose=init_pose,name=joint_str).getValue()
    cont, IK = invKin(p_des=p_end,fk=forKin,frame=end_effector,j_str=joint_str,q_init=q_0,animate=True,T=2)
    # cont, IK = invKin(fk=forKin,j_str=joint_str,q_init=q_init,p_des=p_end,animate=True,T=1)
    print IK
    print cont