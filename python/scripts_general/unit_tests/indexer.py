import numpy as np
import numpy.matlib as ml
from casadi import *

M = 2
N_vec = [120, 30]
N_tot = np.sum(N_vec)
Tf0_vec = [3, 1]
Tf_lb = [3, 0.1]
Tf_ub = [3,3]
W_tau = [1.0, 0.5]
print W_tau

tau_lim_orange = 147.
tau_lim_yellow = 147.
tau_lim = np.array([tau_lim_orange, tau_lim_yellow],ndmin=2).transpose()
tau_lim = np.matlib.repmat(tau_lim,1,M)
print tau_lim
ubtau = tau_lim*W_tau
lbtau = ubtau*-1

for Mi in range(M):
    ubtau_Mi_k = ubtau[:,Mi].tolist()
    print ubtau_Mi_k