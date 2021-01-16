%% Dynamics
% This documents generates two different dynamics model depending on the
% flagReduced. 
% unreduced(false): [qx, q1, q2, q3] of satyrr (q1 is absolute)
% reduced(true): starts with [qx, qw, q1, q2, q3] -> [qx, qa1, q2, q3] (qa1: absolute) 
% 
% The unreduced model requires manual modification such as adding Iw/R^2.
% Result (main_reduced_vs_unreduced) shows that Coriolis terms of two
% models are different. 
clear; clc; 
flagWriteFcn = true;
flagReduced = true;    % reduce the model

%% Generate Rigid Body System
robot = gen_satyrr_full(flagReduced );

%% Calculate (Un)Reduced Dynamics of the Rigid Body System
robot = get_dynamics(robot, flagWriteFcn, flagReduced);

%% Save
cd dynamics
if flagReduced
    suffix = 'reduced';
else
    suffix = 'unreduced';
end
file_name = ['robot_dynamics_', suffix, '.mat'];
save(file_name, 'robot');
cd ..
disp('- saved robot dynamics')

%% Generate Contact Jacobian
% robot = gen_contact(robot, flagWriteFcn);

%% Generate Transmission D/G
% robot = gen_transmission(robot, flagWriteFcn);