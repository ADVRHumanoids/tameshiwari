

#   This script is part of Tameshiwari Repository
#   Git: https://github.com/ADVRHumanoids/tameshiwari.git
#   Created by: Paul Janssen @ Istituto Italiano di Tecnologia
#

#==============================================================================
#       Released packages
#==============================================================================

import numpy as np
import numpy.matlib as ml

#==============================================================================
#       Custom packages
#==============================================================================

import joint_state as js
import functions as fn

#==============================================================================
#       Content
#==============================================================================

nj      = 2
j_name  = []
for j in range(nj):
    tmp     = "J%02d" %j
    j_name.append(tmp)

pose    = fn.RobotPose(j_name)

print pose.name
print pose.q
print pose.qdot
print pose.tau

T       = 3.
f       = 10        # publish frequency
h       = T/f
N       = int(T*f)
lbq     = [-np.pi*5/12, -0.3]
ubq     = [np.pi, 0.3]
q       = np.zeros([N+1,2])
for i in range(nj):
    q[:,i]     = np.matlib.linspace(lbq[i],ubq[i],N+1)
pose.q  = q
pose.rate = f
print pose.q
js.posePublisher(pose)
    