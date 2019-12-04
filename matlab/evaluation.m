clearvars; close all; clc

addpath('/home/user/Matlab/Casadi/casadi-linux-matlabR2014b-v3.4.5')
import casadi.*

addpath('/home/user/catkin_ws/src/tameshiwari/python/OCP_centauro_7dof_arm/casadi_urdf')
filename = 'serialized_function_6dofArm.mat';

load(filename)

invDyn = Function.deserialize(inverse_dynamics)
forKin = Function.deserialize(forward_kinematics)
jacEE = Function.deserialize(jacobian_end_effector)
inertiaJS = Function.deserialize(ineratia_joint_space)
