from casadi import *
import numpy as np
import rospy
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

A = DM(np.ones([2,2])*1)
b = DM(np.ones([2,1])*4)
print -A
print b
print -dot(b,b)
print mtimes(A,b)
print sign(A)

A_2 = mtimes(A,A)
print A_2

A_3 = mtimes(mtimes(A,A),A)
# A_3 = A # A
print A_3

# print float("210"*3)

urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
B_string = kindyn.crba()
B_js = Function.deserialize(B_string)

qk = MX.sym('q_0',2)
B_N = B_js(q=qk)['B']
print B_N


q_0 = [90, 30]
q_0 = np.deg2rad(q_0).tolist()
print q_0
B_0 = B_js(q=q_0)['B']
jacEE_string = kindyn.jacobian('EE')
jacEE = Function.deserialize(jacEE_string)
print jacEE
J_0 = jacEE(q=q_0)['J']
print "B at config q_0 = [0,0]: %s" %B_0
print "Jacobian at config q_0: %s" %J_0

print mtimes(J_0,mtimes(inv(B_0),J_0.T))
