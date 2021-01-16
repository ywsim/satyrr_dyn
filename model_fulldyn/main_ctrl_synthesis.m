% Init
addpath(genpath(pwd));
clear; clc;
load_ = load('robot_dynamics_reduced.mat');  % from gen_floating_leg.m
robot = load_.robot;

% Define Equation of Motion
get_EoM();
% % H * qddot + C = tau = V * u
% H = robot.reduced.H_q;
% C = robot.reduced.C_q;
% K_constraint = robot.reduced.K_con;
% V = K_constraint(:, 3:5);

% Linearize System, 
linearize_EoM()
% % qddot = Z(q, qdot) + Y(q, u)  -> (A, B)
% syms u [robot.nd 1] real
% robot.u = u;
% disp("beginning symbolic simplification of H, C, Z, Y")
% H = simplify(robot.H);      disp("simplified H")
% C = simplify(robot.C);      disp("simplified C")
% Z = simplify(-H\C);         disp("simplified Z")
% Y = simplify(H\robot.u);    disp("simplified Y")
% Acmm_lower = simplify(robot.centr.Acmm);    disp("simplified Acmm")
% Jcm_lower = simplify(robot.ret.Jcm);        disp("simplified Jcm")
% 
% 
% % Make it a function
% m_list = import_m_list_lower();
% cd fcns
% write_fcn_m('fcn_Acmm_lower.m', {'s'}, m_list.s, {Acmm_lower, 'Acmm_lower'});
% write_fcn_m('fcn_Jcm_lower.m', {'s'}, m_list.s, {Jcm_lower, 'Jcm_lower'});
% cd ..
% 
% % Linearization
% robot.s = [robot.q;robot.qd];
% robot.q_0 = [-pi/3; -pi/3; pi/6]; % pose (bent)
% robot.qd_0 = zeros(robot.NB,1); % velocity (rest)
% robot.s_0 = [robot.q_0; robot.qd_0];
% robot.u_0 = double(vpa(subs(C, robot.s, robot.s_0), 6)); % stationary torque
% Z_der_s = jacobian(Z, robot.s);
% Y_der_s = jacobian(subs(Y, robot.u, robot.u_0), robot.s);
% Y_der_u = inv(double(subs(H, robot.s, robot.s_0)));
% 
% % A and B matrices
% A1 = [zeros(robot.NB), eye(robot.NB)];
% A2 = double(subs(Z_der_s + Y_der_s, robot.s, robot.s_0));
% robot.A_0 = [A1; A2];
% B1 = zeros(robot.NB);
% B2 = Y_der_u;
% robot.B_0 = [B1; B2];


% call lqr
