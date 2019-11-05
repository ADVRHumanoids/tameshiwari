import numpy as np
import numpy.matlib as ml
from casadi import *

# M = 2
# N_vec = [120, 30]
# N_tot = np.sum(N_vec)
# Tf0_vec = [3, 1]
# Tf_lb = [3, 0.1]
# Tf_ub = [3,3]
# W_tau = [1.0, 0.5]
# print W_tau

# tau_lim_orange = 147.
# tau_lim_yellow = 147.
# tau_lim = np.array([tau_lim_orange, tau_lim_yellow],ndmin=2).transpose()
# tau_lim = np.matlib.repmat(tau_lim,1,M)
# print tau_lim
# ubtau = tau_lim*W_tau
# lbtau = ubtau*-1

# for Mi in range(M):
#     ubtau_Mi_k = ubtau[:,Mi].tolist()
#     print ubtau_Mi_k

N = 3
M = 2           # multi-stage coefficient
N_stage = [1, 2]
N_tot = np.sum(N_stage)
nq = 3
nj = 2

# ind = np.eye(nq+1)
# for i in range(nq+1):
#     for m in range(M):
#         indnq = list(np.ones(2)*ind[0,i])
#         indnqdot = list(np.ones(nj)*ind[1,i])
#         indnqddot = list(np.ones(nj)*ind[2,i])
#         indh = list(np.ones(1)*ind[3,i])
#         indnstate = list(np.array([indnq,indnqdot,indnqddot]).reshape(-1,1).flatten())
#         tmp = np.matlib.repmat(indnstate,1,N_stage[m]+1).flatten()
#         # print tmp
#         tmp = np.concatenate((np.asarray(indh),tmp)).reshape(-1,1) > 0
#         # print tmp
#         if m == 0:
#             tmp_stage = tmp
#         if m > 0:
#             tmp_stage = np.concatenate((tmp_stage,tmp),axis=0)
#     if i == 0:
#         indexer = tmp_stage
#     else:
#         indexer = np.concatenate((indexer,tmp_stage),axis=1)

# print indexer
# ind = np.eye(nq+1)
# for i in range(nq+1):
#     indnq = list(np.ones(nj)*ind[0,i])
#     indnqdot = list(np.ones(nj)*ind[1,i])
#     indnqddot = list(np.ones(nj)*ind[2,i])
#     indh = list(np.ones(1)*ind[3,i])
#     indnstate = list(np.array([indnq,indnqdot,indnqddot]).reshape(-1,1).flatten())
#     tmp = np.matlib.repmat(indnstate + indh,1,N).flatten()
#     tmp = np.concatenate((tmp,np.asarray(indnstate))).reshape(-1,1) > 0
#     if i == 0:
#         indexer = tmp
#     else:
#         indexer = np.concatenate((indexer,tmp),axis=1)

# a = DM([1,3])
# # a[0] = 1
# # a[1] = 3
# print a
# print sum2(a)
N = 20
factor = np.ones(20)
print factor
factor[-5:] = np.matlib.linspace(1,0,5)
print factor