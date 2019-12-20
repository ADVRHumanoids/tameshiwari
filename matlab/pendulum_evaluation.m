clearvars; close all; clc;

addpath('../python/OCP_pendulum_2dof_rr/results');
filename = 'rr_robot_max_velocity_mutli_stage_final_2019-12-20T11:21:08.mat';
load(filename)

if isunix
%   path for Linux machine
    save_path = '/home/user/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter3/';
    addpath('/home/user/catkin_ws/src/tameshiwari/matlab')
elseif ismac
%   path for macbook
    save_path = '/Users/Paul/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter3/';
    addpath('/Users/Paul/Documents/GitHub/tameshiwari/matlab')
end

ccycle = colorcycle();
pt = 426.7913;
inch = pt*0.01384;

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
stairs(time,joint_effort.tau_lb','--')
stairs(time,joint_effort.tau_ub','--')
xlim([0 time(end)])

figure(4)
plot(time,pdot_e(3,:))
hold on
plot([time(1) time(end)],[-6.19 -6.19])
xlim([0 time(end)])