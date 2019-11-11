import os, sys
from casadi import *
import rospy
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

urdf = rospy.get_param('robot_description')

dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0] +  '/casadi_urdf'
fileName = "%s/centauro_2dof_urdf.txt" % dirName

write_new = False
if write_new:
    with open(fileName, 'w+') as f:
        f.write(urdf)

with open(fileName, 'r') as f:
    urdf = f.read()

kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
fk_string = kindyn.fk('arm2_8')
forKin = Function.deserialize(fk_string)
print forKin