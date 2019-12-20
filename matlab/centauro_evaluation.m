clearvars; close all; clc;

addpath('../python/OCP_centauro_7dof_arm/results');
filename = 'centauro_max_momentum_7dof_final_2019-12-20T12:07:11.mat';
load(filename)

if isunix
%   path for Linux machine
    save_path = '/home/user/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter4/';
    addpath('/home/user/catkin_ws/src/tameshiwari/matlab')
elseif ismac
%   path for macbook
    save_path = '/Users/Paul/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter4/';
    addpath('/Users/Paul/Documents/GitHub/tameshiwari/matlab')
end

figure(1)
plot(time,joint_position.q)
hold on
plot(time,joint_position.q_lb,'--')
plot(time,joint_position.q_ub,'--')
xlim([0 time(end)])

figure(2)
plot(time,joint_velocity.qdot)
hold on
plot(time,joint_velocity.qdot_lb,'--')
plot(time,joint_velocity.qdot_ub,'--')
xlim([0 time(end)])

figure(3)
stairs(time,joint_effort.tau')
hold on
stairs(time,joint_effort.tau_lb,'--')
stairs(time,joint_effort.tau_ub,'--')
xlim([0 time(end)])

figure(4)
plot(time,h_e_scalar)
hold on
xlim([0 time(end)])