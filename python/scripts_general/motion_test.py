

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#

#==============================================================================
#       Released packages
#==============================================================================

import numpy as np
import numpy.matlib as ml
import matplotlib.pyplot as plt

#==============================================================================
#       Custom packages
#==============================================================================

import joint_state as js
import functions as fn

#==============================================================================
#       Content
#==============================================================================

# nj      = 2
# j_name  = []
# for j in range(nj):
#     tmp     = "J%02d" %j
#     j_name.append(tmp)


# T       = 2.735
# f       = 10            # publish frequency
# h       = T/f
# N       = int(T*f)
# print "N is %s" %N
# lbq     = [-np.pi*5/12, -0.3]
# ubq     = [np.pi, 0.3]
# q       = np.zeros([N+1,2])
# for i in range(nj):
#     q[:,i]     = np.matlib.linspace(lbq[i],ubq[i],N+1)

# print q
# pose = fn.RobotPose(j_name,q,[],[],f)
# print "Joint name vector: %s" %pose.name
# print "N is %s" % pose.N

# print N/f

# N = 11
# qi = 10
# qf = 40
# qdoti = 0
# qdotf = -5
# ti = 2
# tf = 4
# # Ax = b
# a0 = [1, ti, ti**2, ti**3]
# a1 = [0, 1, 2*ti, 3*ti**2]
# a2 = [1, tf, tf**2, tf**3]
# a3 = [0, 1, 2*tf, 3*tf**2]
# A = np.array([a0,a1,a2,a3])
# print A
# b = np.array([qi,qdoti,qf,qdotf])
# print b
# x = np.flipud(np.linalg.solve(A,b))
# print x

# # print np.allclose(np.dot(A,x),b)

# T = ml.linspace(ti,tf,N)
# # print T
# y = np.zeros([N])
# # print y
# # print range(N)
# for i in range(N):
#     # tmp = 
#     # print tmp
#     y[i] = np.polyval(x,T[i])
# print y

# ymat = np.zeros([N,3])
# plus = range(3)
# for j in range(3):
#     ymat[:,j] = y + plus[j]
# print ymat
# print np.transpose(ymat)
# plt.figure(1)
# plt.clf()
# plt.plot(T,ymat)
# plt.show()


# js.posePublisher(pose)
# testq  = np.zeros([N+1,2])
# test = fn.RobotPose(j_name,testq)

# # print test.nj
# print np.shape(test.q)
# print np.shape(q)
# print np.shape(test.q)[0] == np.shape(q)[0]

# print test.rate
# print test.T

# interval = np.array([1.2104, 1.2326, 1.3211, 1.4261])
# print np.nonzero((interval>1.3)&(interval<1.4))[0]
