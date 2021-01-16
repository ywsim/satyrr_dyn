%% Init
clear; clc; close all;
flagWriteFcn = true;
flagReduced = false;    % reduce the model

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