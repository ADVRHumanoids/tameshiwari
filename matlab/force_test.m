clearvars; close all; clc;


% 10CM AWAY FROM THE SINGULARITY
p_e = [0.90176, 0.000686027, 1.24463]'
pdot_e = [0.288391, -0.458663, -5.6322]'
omega_e = [1.54014, 10.4654, 0.480762]'

J_e =   [[-0.0240074, 0.166623, 0.169014], 
         [0.144664, 0.416364, 0.097376], 
         [-0.642438, 0.164814, -0.416866], 
         [0.492404, 0.640901, 0.348735], 
         [0.852869, -0.490284, 0.871432], 
         [0.173648, 0.590651, 0.344949]];

gamma_e = [0,0,1,0,0,0]'

tau_iota = J_e'*gamma_e