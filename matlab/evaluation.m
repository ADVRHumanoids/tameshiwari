clearvars; close all; clc

%% Import Casadi depending on the system which is running
try
%   path for Linux machine
    addpath('/home/user/Matlab/Casadi/casadi-linux-matlabR2014b-v3.4.5')
catch
%   path for macbook
    addpath('/home')
end
import casadi.*

%% Import Pynocchio functions for evaluating results
addpath('../python/OCP_centauro_7dof_arm/casadi_urdf')
filename = 'serialized_function_6dofArm.mat';

load(filename)

invDyn = Function.deserialize(inverse_dynamics)
forKin = Function.deserialize(forward_kinematics)
jacEE = Function.deserialize(jacobian_end_effector)
inertiaJS = Function.deserialize(ineratia_joint_space)

%% Import results file
addpath('../results/Karate chop arm optimal - with impact  4-dec')
filename = 'XBotCore_log__2019_12_04__10_59_36.mat';

load(filename)

%% Select experiment data
T_lower = 2905.45;
T_upper = 2910;
T_tot = T_upper-T_lower;
select = find(time>T_lower & time<T_upper);
data_points = size(select,2);
% plot(time(select),joint_velocity(left_arm,select))

joints = left_arm(1:6);
nj = numel(joints);
T = linspace(0,T_tot,data_points);
q_opt = joint_position(joints,select);
qdot_opt = joint_velocity(joints,select);
qddot_opt = joint_acceleration(joints,select);
tau_opt = joint_effort(joints,select);
mu = 10^(-2);
u_hat = [0;0;1;0;0;0];

figure(1)
for i = 1:6
    subplot(3,2,i)
    plot(T,qdot_opt(i,:))
end

% xlim([T_lower T_upper])

%% Calculate momentum
twist_opt = zeros(6,data_points);
jacobian_opt = reshape(zeros(6*nj*data_points,1),[6,nj,data_points]);
joint_inertia_opt = reshape(zeros(nj*nj*data_points,1),[nj,nj,data_points]);
task_inertia_opt = reshape(zeros(6*6*data_points,1),[6,6,data_points]);
momentum_vector_opt = zeros(nj,data_points);
momentum_scalar_opt = zeros(1,data_points);

for k = 1:data_points
    q_k = q_opt(:,k);
    qdot_k = qdot_opt(:,k);
    jacobian_k = jacEE('q',q_k);
    jacobian_k = full(jacobian_k.J);
    twist_k = jacobian_k*qdot_k;
    joint_I_k = inertiaJS('q',q_k);
    joint_I_k = full(joint_I_k.B);
    task_I_k = inv(jacobian_k*inv(joint_I_k)*jacobian_k' + mu*eye(6));
    h_vector_k = task_I_k*twist_k;
    h_scalar_k = u_hat'*h_vector_k;
    
    twist_opt(:,k) = twist_k;
    jacobian_opt(:,:,k) = jacobian_k;
    joint_inertia_opt(:,:,k) = joint_I_k;
    task_inertia_opt(:,:,k) = task_I_k;
    momentum_vector_opt(:,k) = h_vector_k;
    momentum_scalar_opt(k) = h_scalar_k;
end

figure(2)
plot(T,momentum_scalar_opt)

%% Import simulation file
% clearvars
addpath('../results/Karate chop arm optimal - with impact  4-dec')
filename = 'RobotPose_centauro_max_momentum_7dof_v1_2019-12-04T10:00:09.mat';

load(filename)
joints = 1:6;
data_points = numel(q(:,1));
nj = numel(joints);
q_opt = q(:,joints)';
qdot_opt = qdot(:,joints)';
% qddot_opt = qddot(:,joints)';
% tau_opt = tau(:,joints)';
dt = 1/30;
T_tot = dt*(data_points-1);
T = linspace(0,T_tot,numel(q(:,1)));
figure(1)
for i = 1:6
    subplot(3,2,i)
    hold on;
    plot(T,qdot_opt(i,:))
end

%% Calculate momentum
twist_opt = zeros(6,data_points);
jacobian_opt = reshape(zeros(6*nj*data_points,1),[6,nj,data_points]);
joint_inertia_opt = reshape(zeros(nj*nj*data_points,1),[nj,nj,data_points]);
task_inertia_opt = reshape(zeros(6*6*data_points,1),[6,6,data_points]);
momentum_vector_opt = zeros(nj,data_points);
momentum_scalar_opt = zeros(1,data_points);

for k = 1:data_points
    q_k = q_opt(:,k);
    qdot_k = qdot_opt(:,k);
    jacobian_k = jacEE('q',q_k);
    jacobian_k = full(jacobian_k.J);
    twist_k = jacobian_k*qdot_k;
    joint_I_k = inertiaJS('q',q_k);
    joint_I_k = full(joint_I_k.B);
    task_I_k = inv(jacobian_k*inv(joint_I_k)*jacobian_k' + mu*eye(6));
    h_vector_k = task_I_k*twist_k;
    h_scalar_k = u_hat'*h_vector_k;
    
    twist_opt(:,k) = twist_k;
    jacobian_opt(:,:,k) = jacobian_k;
    joint_inertia_opt(:,:,k) = joint_I_k;
    task_inertia_opt(:,:,k) = task_I_k;
    momentum_vector_opt(:,k) = h_vector_k;
    momentum_scalar_opt(k) = h_scalar_k;
end

figure(2)
hold on;
plot(T,momentum_scalar_opt)







