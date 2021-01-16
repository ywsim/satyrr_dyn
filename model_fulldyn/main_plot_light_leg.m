%% Intro
% This document verifies if the  directional floating-body robot's GIE
% maxes out by the total mass of the robot. Analyzed a planar, 5-DoF,
% floating, serial robot on xy-plane. The first 3-DoF consist of px, py, rz of
% the torso(3), followed by 2-DoF of rz's of hip and knee. 

%% Init
clear; clc; 
addpath(genpath(pwd))

% Import Dynamics 
load_dyn = load('robot_dynamics.mat');  % from gen_floating_leg.m
robot_dyn = load_dyn.robot;

% Import Numerical Values of L, m, I
params_value = import_robot_params('value-light');
L = params_value.L;
m = params_value.m;
I = params_value.I;

%% Figure
figure()
sgtitle('Vertical Mass vs Hip Angle, Floating 2 DoF')
limy = [0 sum(m)*1.1];
limx = [ -90 0 ];

%% Variables and Parameters
% angles
sz_q = 200;                  % array size
qx = zeros(1, sz_q);        % torso coordinate 1 (set to zero, trivial)
qy = qx;                    % torso coordinate 2 (set to zero, trivial)
q1 = qx;                    % torso coordinate 3 (set to zero, trivial)
q2 = linspace(0, -pi/2, sz_q);     % hip angle
q3 = -pi - 2*q2;             % knee angle
% q3 = -pi/2 - q2;             % knee angle
q = [qx; qy; q1; q2; q3];   

% gears
Irot1 = 0.0023/6^2;                % Inertia of Rotor
Irot2 = Irot1;
N1 = 5;                     % Gearing Ratio
N2 = N1;
gear = [Irot1; Irot2; N1; N2];          

% efficiency
sz_eff = 100;                % array size 
n_fwd1 = linspace(0.5, 1, sz_eff);     % forward eff array
n_fwd2 = n_fwd1;
n_bwd1 = fcn_f2b_eff(n_fwd1, N1);     % backward eff array
n_bwd2 = fcn_f2b_eff(n_fwd2, N2);
% n_bwd1 = linspace(0.5, 1, sz_eff);
% n_bwd2 = n_bwd1;
eff = [n_bwd1;n_bwd2;n_fwd1;n_fwd2];    

% intercept
unit_vec = [0; 1];          % directional task-space inertia's direction
origin = [0;0];
scale = 1;
%% subplot
subplot(3,1,1)
scenario = 'fwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');
    end
end

for idx_eff = 1:sz_eff
    color = 0.7*(1- 1*idx_eff/sz_eff) ;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(3,1)*color)
    hold on
end

title(['FWD, N = ', num2str(N1)])
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

subplot(3,1,3)
qx_sample = 0;
qy_sample = 0;
q1_sample = 0;
q2_sample = -pi/3;
q3_sample = -pi/3;
q_sample = [qx_sample, qy_sample, q1_sample, q2_sample, q3_sample];
eff_sample = [0.75, 0.75, fcn_f2b_eff(.75, N1), fcn_f2b_eff(.75, N2)];
Zf = calcAGIE(q_sample, L, m, I, gear, eff_sample, scenario, drive);

plot_ellip(gca, Zf, origin, scale);
hold on

%% subplot
subplot(3,1,2)
scenario = 'bwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');        
    end
end

for idx_eff = 1:sz_eff
    color = 0.7*(1- 1*idx_eff/sz_eff) ;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(1,3)*color)
    hold on
end

title(['BWD, N = ', num2str(N1)])
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

subplot(3,1,3)
qx_sample = 0;
qy_sample = 0;
q1_sample = 0;
q2_sample = -pi/3;
q3_sample = -pi/3;
q_sample = [qx_sample, qy_sample, q1_sample, q2_sample, q3_sample];
eff_sample = [fcn_f2b_eff(.6, N1), fcn_f2b_eff(.6, N2), 0.6, 0.6];
Zb = calcAGIE(q_sample, L, m, I, gear, eff_sample, scenario, drive);

plot_ellip(gca, Zb, origin, scale);



%% subplot
subplot(3,1,3)
% nogear = zeros(4,1);
eff_ideal = ones(4,1);
Zi = calcAGIE(q_sample, L, m, I, gear, eff_ideal, 'bwd', drive);
plot_ellip(gca, Zi, origin, scale);
legend('fwd', 'bwd', 'idl')
daspect(ones(3,1))

set(gcf,'Position',[100 100 500 800])
