function ubar = QP_stabilizing_controller(QP)
%% QP solver
% description 

%% Optimization variables
% Optimization variables: x = [ubar;F_ext;qdd]
ubar_num = 3;
F_ext_num = 3;
qdd_num = 6;
centr_coord_num = 6;
opt_var_num = ubar_num + F_ext_num + qdd_num;

%% QP set-up data
xbar = QP.xbar;
q_qd = QP.q_qd;
qd_idx = QP.qd_idx;
qd = q_qd(qd_idx);
H = QP.H;
C = QP.C;
S_ext_partial = QP.S_ext_partial;
Acmm = QP.Acmm;
Acmm_dot = QP.Acmm_dot;
mg = QP.mg;
u_nom = QP.u_nom;
u_nom_lower = QP.u_nom(1:3);
u_arm = QP.u_arm;
u_stall = QP.u_stall;
LQR_ubar = QP.LQR_ubar;
F_ext_idx = QP.F_ext_idx;

%% Cost function
% R matrice - penalize control effort
u_weights = [10, 100, 10];
R = 100* diag(u_weights);

% SB matrice - penalize coupling cost
SB = .1*[eye(ubar_num), 1*zeros(ubar_num)];

% cost function: (1/2)*x^T*H*x + f^T*x -> ubar^T*R*ubar + (2*xbar^T*SB)ubar
H_cost_u = R;
H_cost_F = ones(F_ext_num);
H_cost_qdd = blkdiag(ones(3,3), 10*ones(3,3));

H_cost = 2*blkdiag(H_cost_u, H_cost_F, H_cost_qdd);
f = [2*SB*xbar; ones(opt_var_num - ubar_num, 1)];

%% Friction
% inequality constraints: A*x <= b
A = [];
b = [];

%% Dynamics (manipulator equations and centroidal dynamics)
% equality constraints: Aeq*x = beq

% create S_ext and Bu matrices (fix hard-coding of indices later)
S_ext = zeros(centr_coord_num, F_ext_num);
S_ext(F_ext_idx, :) = S_ext_partial;
H1 = H(1:3, :);
C1 = C(1:3);
Bu1 = eye(3, 3);
H2 = H(4:6, :);
C2 = C(4:6);
Bu2 = eye(3);
zero_mat_12 = zeros(3, 3);
zero_mat_21 = zeros(3, 3);
zero_mat_22 = zeros(3, 3);
zero_mat_31 = zeros(6, 3);

% dynamics
Aeq = [-Bu1, zero_mat_12, H1; 
    zero_mat_21, zero_mat_22, H2;
    zero_mat_31, -1*S_ext, Acmm];
beq = [(-C1 + Bu1*u_nom_lower); 
    (-C2 + Bu2*u_arm);
    (-Acmm_dot*qd + mg)];

%% input limits (motor torque, ground reaction forces)
u_stall = 100;
% lower bounds: lb
ubar_min = -u_stall*ones(ubar_num, 1) - u_nom_lower;
F_bx_min = -Inf;
F_by_min = 0;
F_fy_min = 0;
F_ext_min = [F_bx_min; F_by_min; F_fy_min];
% qdd_min = -Inf*ones(qdd_num, 1);
qdd_min = -500*ones(qdd_num,1);
lb = [ubar_min; F_ext_min; qdd_min];

% upper bounds: ub
ubar_max = u_stall*ones(ubar_num, 1) - u_nom_lower;
F_ext_max = Inf*ones(F_ext_num, 1);
% qdd_max = Inf*ones(qdd_num, 1);
qdd_max = 500*ones(qdd_num,1);
ub = [ubar_max; F_ext_max; qdd_max];

%% Use LQR control as initial guess - assume no knowledge of disturbance
% initial guess: x0
% use LQR torque
ubar0 = LQR_ubar;

tau = u_nom + [ubar0; u_arm];
x_dot = dynStickMan(0, q_qd, tau);
qdd0 = x_dot(qd_idx);
F_rem = Acmm*qdd0 + Acmm_dot*qd - mg;
F_con = S_ext_partial\F_rem(F_ext_idx);
F_ext0 = zeros(F_ext_num, 1);
% disp(F_con)
for F_ext_idx_loop = 2:3
    if F_con(F_ext_idx_loop) > 0
        F_ext0(F_ext_idx_loop) = F_con(F_ext_idx_loop);
    end
end
x0 = [ubar0; F_ext0; qdd0];

%% solve qp
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'MaxIterations', 500, 'ConstraintTolerance', 1E-4,'Display','off');
[opt_var, ~, ExitFlag] = quadprog(H_cost, f, A, b, Aeq, beq, lb, ub, x0, options);

disp(['ExitFlag: ', num2str(ExitFlag)])
% conIneq
conEq = transpose(Aeq*opt_var - beq)
conLowerBound = transpose(opt_var > lb)
conUpperBound = transpose(opt_var < ub)
%% get ubar
ubar = opt_var(1:ubar_num);
qdd = opt_var(ubar_num + F_ext_num + 1: end);
Fcon = opt_var(ubar_num + 1 : ubar_num+F_ext_num);
disp(ubar')
disp(qdd')
disp(Fcon')

end