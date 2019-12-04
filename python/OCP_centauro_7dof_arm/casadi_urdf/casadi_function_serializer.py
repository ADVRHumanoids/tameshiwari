#!/usr/bin/env python2

#==============================================================================
#   RELEASED PACKAGES
#==============================================================================

from casadi import *
import os, sys
import numpy as np
import numpy.matlib as ml
import scipy.io as sio
import rospy

#==============================================================================
#   CUSTOM PACKAGES
#==============================================================================

import functions as fn
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

load_file = True
dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0]

if load_file:
    torso_fixed = False
    if torso_fixed:
        fileName = "%s/centauro_urdf_6dof_joints_1111110.txt" % dirName
    else:
        fileName = "%s/centauro_urdf_7dof_joints_1111110_torso_test.txt" % dirName
    with open(fileName, 'r') as f:
        urdf = f.read()
        # print urdf
else:
    urdf = rospy.get_param('robot_description')

# print urdf
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

end_effector = 'arm1_8'
invDyn = kindyn.rnea()
forKin = kindyn.fk(end_effector)
jacEE = kindyn.jacobian(end_effector)
inertiaJS = kindyn.crba()
printer = False
if printer:
    print invDyn
    print forKin
    print jacEE
    print inertiaJS

message = 'j_armX_7 is fixed on both arms and the 2dof in the neck are fixed and velodyne missing'
message = ''

funString = {
    'urdf': urdf,
    'inverse_dynamics': invDyn,
    'forward_kinematics': forKin,
    'jacobian_end_effector': jacEE,
    'ineratia_joint_space': inertiaJS,
    'joint_message': message
}

fileName = "serialized_function_6dofArmPlusTorso.mat"
sio.savemat(dirName + '/' + fileName,funString)

invDyn = Function.deserialize(kindyn.rnea())
forKin = Function.deserialize(kindyn.fk(end_effector))
jacEE = Function.deserialize(kindyn.jacobian(end_effector))
inertiaJS = Function.deserialize(kindyn.crba())

printer = True
if printer:
    print invDyn
    print forKin
    print jacEE
    print inertiaJS