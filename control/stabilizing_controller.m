function ctrl = stabilizing_controller(robot, ctrl_type)

% LQR matrices
% scaling variables
w1 = 1;
w2 = 10;
w3 = 10;
% penalize individual joint position deviation
Q1q = [1, 1, 1];
Q1qd = [0.5, 0.5, 0.5];
Q1 = diag([Q1q, Q1qd]);
Q_Q1 = Q1;
% penalize center of mass position deviation
Q2cm = [1 1 1];
Q2 = diag(Q2cm);
Jcm_s_0 = fcn_Jcm_lower(robot.s_0);
Q_Q2 = [Jcm_s_0'*Q2*Jcm_s_0, zeros(robot.nd, robot.nd); zeros(robot.nd, 2*robot.nd)];
% penalize centroidal momentum deviation
Q3cmm = [1 1 1 1 1 1];
Q3 = diag(Q3cmm);
Acmm_s_0 = fcn_Acmm_lower(robot.s_0);
Q_Q3 = [zeros(robot.nd, 2*robot.nd); zeros(robot.nd, robot.nd), Acmm_s_0'*Q3*Acmm_s_0];
% final Q matriz
Q = w1*Q_Q1 + w2*Q_Q2 + w3*Q_Q3;
% penalize control effort
u_weights = [10, 100, 10];
R = diag(u_weights);

% Calculate gain matrix
K = lqr(robot.A_0, robot.B_0, Q, R);

% Store LQR info
LQR.Q = Q;
LQR.R = R;
LQR.K = K;
ctrl.LQR = LQR;

% Initial state and control
ctrl.s_0 = robot.s_0;
ctrl.u_0 = robot.u_0;

% Motor stall torque
ctrl.u_stall = 0.39;

% set control type (LQR or QP)
ctrl.ctrl_type = ctrl_type;

end