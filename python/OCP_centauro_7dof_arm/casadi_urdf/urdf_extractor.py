import os, sys
from casadi import *
import rospy
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_1dof_joints_1000000.txt'	1	1	0	0	0	0	0	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_2dof_joints_1001000.txt'	2	1	0	0	1	0	0	0 done 
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_3dof_joints_1101000.txt'	3	1	1	0	1	0	0	0 done 
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_3dof_joints_1011000.txt'	3	1	0	1	1	0	0	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_4dof_joints_1101010.txt'	4	1	1	0	1	0	1	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_4dof_joints_1011010.txt'	4	1	0	1	1	0	1	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_5dof_joints_1111010.txt'	5	1	1	1	1	0	1	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_5dof_joints_1101110.txt'	5	1	1	0	1	1	1	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_5dof_joints_1011110.txt'	5	1	0	1	1	1	1	0 done
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_6dof_joints_1111110.txt'	6	1	1	1	1	1	1	0 done 
# name	                                 DoF   sp  sr  sy	e  fy  fp  wy
# 'centauro_urdf_7dof_joints_1111111.txt'	7	1	1	1	1	1	1	1 done

dirName = os.path.split(os.path.abspath(os.path.realpath(sys.argv[0])))[0]
if "casadi_urdf" not in dirName:
    sys.exit('####### Navigate to folder location of this file #######\n')

fileName = 'centauro_urdf_7dof_joints_1111111.txt'
fileName = dirName + "/" + fileName
print fileName

urdf = rospy.get_param('robot_description')
print urdf
with open(fileName, 'w+') as f:
        f.write(urdf)

with open(fileName, 'r') as f:
    urdf_read = f.read()

kindyn = cas_kin_dyn.CasadiKinDyn(urdf_read)
fk_string = kindyn.fk('arm1_8')
forKin = Function.deserialize(fk_string)
print forKin