%% Compare two models of different approaches
% unreduced: [qx, q1, q2, q3] of satyrr (q1 is absolute)
% reduced: starts with [qx, qw, q1, q2, q3] -> [qx, qa1, q2, q3] (qa1: absolute) 
clear; clc; 
addpath(genpath(pwd))

% Import Dynamics 
load_dyn_reduced = load('robot_dynamics_reduced.mat');  % from gen_floating_leg.m
load_dyn_unreduced = load('robot_dynamics_unreduced.mat');  % from gen_floating_leg.m
robot_reduced = load_dyn_reduced.robot;
robot_unreduced = load_dyn_unreduced.robot;

H_u = robot_unreduced.H_q;              % unreduced model's H
C_u = robot_unreduced.C_q;              % unreduced model's C
H_r_temp = robot_reduced.reduced.H_q;   % reduced model's H
C_r_temp = robot_reduced.reduced.C_q;   % reduced model's C
syms q1 qa1 qd1 qda1 real
H_r = subs(H_r_temp, qa1, q1);          % change of variable qa1 -> q1
C_r = subs(C_r_temp, [qa1, qda1], [q1, qd1]); % change of variable qa1, qda1 -> q1, qd1

deltaH = simplify(expand(H_u - H_r))    % difference = 0
deltaC = simplify(expand(C_u - C_r))    % difference != 0
