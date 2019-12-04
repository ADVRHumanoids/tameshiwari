clearvars; close all; clc;

A = ones(2)
A(2,1) = 0
rank(A)


% J = [[-1.00393e-05, 0], 
%  [0.777951, 0.505184], 
%  [0.104221, 0], 
%  [1, 1], 
%  [0, 0], 
%  [9.63268e-05, 9.63268e-05]]

% The optimized final EE cartesian position: [0.08, 0.780998, 6.90085e-05] [m]

J = [[-7.5231e-05, -4.80954e-05], 
 [-6.13024e-05, -0.0769184], 
 [0.780998, 0.499294], 
 [1, 1], 
 [0, 0], 
 [9.63268e-05, 9.63268e-05]]

J = J(2:3,:)

B = [[2.45804, 1.28541], 
 [0, 0.773341]]

B = [[2.45804, 1.28541], 
 [0, 0.773341]]

B_inv = inv(B)

Lambda_inv = J*B_inv*transpose(J)
Lambda = inv(Lambda_inv)
J_plus = pinv(J);
Lambda = J_plus'*B*J_plus