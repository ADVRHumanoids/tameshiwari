clearvars; close all; clc;

addpath('../python/OCP_pendulum_2dof_rr/results');
filename = 'rr_robot_max_velocity_mutli_stage_final_2019-12-20T01:46:58.mat';
load(filename)

figure(1)
plot(time,joint_velocity.qdot)
hold on
plot(time,joint_velocity.qdot_lb,'--')
plot(time,joint_velocity.qdot_ub,'--')
xlim([0 time(end)])