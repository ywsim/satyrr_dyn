clear; clc; 
addpath(genpath(pwd))

% Import Dynamics 
load_dyn_reduced = load('robot_dynamics_reduced.mat');  % from gen_floating_leg.m
load_dyn_unreduced = load('robot_dynamics_unreduced.mat');  % from gen_floating_leg.m
robot_reduced = load_dyn_reduced.robot;
robot_unreduced = load_dyn_unreduced.robot;

H_q_u = robot_unreduced.H_q;
H_q_r_temp = robot_reduced.reduced.H_q
syms q1 qa1 
H_q_r = subs(H_q_r_temp, qa1, q1)

dH = simplify(expand(H_q_u - H_q_r))

