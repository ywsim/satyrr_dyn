function robot = spatial_6_dof_monoped()
%% Description & Init
% This file generates RB model of a planar, fixed-base and serial link robot with 5 joints. 
% While the robot has a foot, the model assumes the foot as the fixed base.
% Bodies: {B} - {1, 2, 3, 4, 5, 6} 
% Planar: {x,z} 
% Robot lies straight on the ground along x-dir. 

%% Init
addpath(genpath(pwd))
clear;clc;

robot.name = '6 DOF 6-Link Monoped (Half-cut-MINI)';
robot.base = 'fixed';
robot.drive = 'serial';
robot.dimensions = 'spatial';
robot.nd = 6;   % number of DoF

%% Import Params
model_prms = model_params_6();
robot.model_prms = model_prms;

%% Rigid Body Params
% Graph Connectivity
robot.parent = [0 1 2 3 4 5];  % legnth: 6

% CoM Position
com_ = [model_prms.Lc, zeros(robot.nd, 2)];    

% mass
mass_ = model_prms.M;
robot.mass_tot = sum(mass_);

% Iner 
iner_ = [zeros(robot.nd, 1), 1/12*model_prms.M.*(model_prms.L.^2), 1/12*model_prms.M.*(model_prms.L.^2)];

% Joint Type
jtype_ = {'Ry', 'Ry', 'Ry', 'Ry', 'Rz', 'Rz'}; % legnth: 4

% Joint Tree Transform 
jpos_=[  zeros(1,3); ... % base (trivial)
            model_prms.L1   0  0;  ... % base   -> gimbal joint origin
            model_prms.L2   0  0;
            model_prms.L3   0  0;
            model_prms.L4   0  0;
            model_prms.L5   0  0;
            ];

% Numbering
for idx_body=1:robot.nd
    robot.jtype{idx_body} = jtype_{idx_body};          % joint type
    robot.com{idx_body}   = com_(idx_body,:);           % Com Position
    robot.mass{idx_body} = mass_(idx_body);                   % Body Mass 
    robot.I{idx_body} = mcI(robot.mass{idx_body}, robot.com{idx_body}, diag(iner_(idx_body,:))); % Inertia
    robot.Xtree{idx_body} = plux(eye(3), jpos_(idx_body,:));   % Tree transform
end

% robot contact points
robot.contact.rWF = model_prms.rWF;
robot.contact.rWB = model_prms.rWB;
   
stateSize = [robot.nd 1];
syms q qd qdd f_ext [robot.nd 1] real
robot.q =q ;
robot.qd = qd;
robot.qdd = qdd;
robot.f_ext = f_ext;  % this is not required one, you can still get H and C without passing external force term 
