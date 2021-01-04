% function robot = planar_3_dof_prp_satyrr()
% Satyrr 3-DoF Schematic Model
% This code generates rigid body model of a fixed-base 3 DoF simplified 
% model of % the Satyrr. 
% The model consist of px-ry-pz actuators. 

%% Init
addpath(genpath(pwd))
clear;clc;

robot.name = '3 DoF 3-Link Planar Satyrr';
robot.base = 'fixed';
robot.drive = 'serial';
robot.dimentions = 'planar';
robot.nd = 3;   % number of DoF

%% Import Params
% model_prms = model_params();
% robot.model_prms = model_prms;
syms p th z real
syms p0 th0 z0  real
syms pd thd zd  real
syms m real

q = [ p; th; z];
qd= [pd; thd; zd];

robot.q = q;
robot.qd = qd;

%% Rigid Body Params
robot.parent = [0 1 2];         % Rigid Body Connectivity

mass = [0; 0; m];         % mass of bodies
mass_tot = sum(mass);
com = [ zeros(1,3);
        zeros(1,3);
        zeros(1,3)];
iner = [  zeros(1,3);     % massless    
          zeros(1,3);     % massless
          zeros(1,3)];    % point mass
            
jtype = {'Px', 'Ry', 'Pz'};
jpos= [ zeros(1,3);          % trivial
        p0 0 0;
        0 0 z0];

for idx_body = 1:robot.nd
   robot.jtype{idx_body} = jtype{idx_body};
   robot.com{idx_body} = com(idx_body,:);
   robot.mass{idx_body} = mass(idx_body);
   robot.I{idx_body} = mcI(robot.mass{idx_body}, robot.com{idx_body}, diag(iner(idx_body,:)));
   robot.Xtree{idx_body} = plux(eye(3), jpos(idx_body, :));
end


%% Calc Dyn
% Number of bodies
robot.NB = robot.nd;

% Use robot model structure and calculate H (inertia) and C (coriolis)
% matrix
[H, C] = HandC(robot, robot.q, robot.qd);
robot.H = simplify(H);
robot.C = simplify(C);