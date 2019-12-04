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

J_ts = J_e([1,3,6],:)
% J_ts = J_e(1:3,:)

B_e =   [[1.17669, -0.115583, 0.487529], 
         [0, 0.857707, -0.00577066], 
         [0, 0, 0.330662]];

B_e = B_e + triu(B_e,1)'

v_e = [pdot_e([1,3]);omega_e(3)]
% v_e = pdot_e

Lambda_e = inv(J_ts*inv(B_e)*J_ts');
h_e = Lambda_e * v_e


% without adding extra term to B and using linear velocity:
% h_e =
% 
%   -17.0812
%     3.1604
%   -10.9889
% without adding extra term to B and using lin & rot velocity:
% h_e =
% 
%   -12.5921
%   -15.1897
%    -2.5170
% with adding extra term to B and using linear velocity:
% h_e =
% 
%    -6.7043
%    -1.4928
%   -12.4245
% 
% with adding extra term to B and using lin & rot velocity:
% h_e =
% 
%    -8.9167
%   -10.2812
%     1.2838
