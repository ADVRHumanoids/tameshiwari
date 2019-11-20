# EMPTY THIS SCRIPT AT THE END OF USAGE

from casadi import *

vector = DM([0,0,1,0,0,0])
print vector
print vector.size()
weight = MX([100])
print weight
W = MX(6,6)
W[5,5] = 1 * weight
tau = MX.ones(6)*3
print tau
print W
print W.size()


print mtimes(tau.T,mtimes(W,tau))