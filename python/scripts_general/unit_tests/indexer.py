import numpy as np
import numpy.matlib as ml
nq = 3
# nq = 4
nj = 2
N = 2

ind = np.eye(nq+1)
for i in range(nq+1):
    indnq = list(np.ones(nj)*ind[0,i])
    indnqdot = list(np.ones(nj)*ind[1,i])
    indnqddot = list(np.ones(nj)*ind[2,i])
    indh = list(np.ones(1)*ind[3,i])
    indnstate = list(np.array([indnq,indnqdot,indnqddot]).reshape(-1,1).flatten())
    tmp = np.matlib.repmat(indnstate + indh,1,N).flatten()
    tmp = np.concatenate((tmp,np.asarray(indnstate))).reshape(-1,1) > 0
    if i == 0:
        indexer = tmp
    else:
        indexer = np.concatenate((indexer,tmp),axis=1)