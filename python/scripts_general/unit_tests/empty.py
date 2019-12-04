# EMPTY THIS SCRIPT AT THE END OF USAGE

import numpy as np

nj = 4
N_tot = 20

q_lb = [0.5, 0.7, 0.4, 0.3]
q_lbopt = np.zeros([nj,N_tot+1])
q_lbopt[:,0] = q_lb
print q_lb
print q_lbopt