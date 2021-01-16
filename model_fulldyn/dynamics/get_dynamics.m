function robot = get_dynamics(robot, flagWriteFcn, flagReduced)

if ~flagReduced     
    %% Unreduced Model
    [H_raw, C_raw] = HandC(robot, robot.q, robot.qd);
    H_q_temp = simplify(H_raw);
    syms R Iw real
    H_q_temp(1) = H_q_temp(1) + Iw / R^2;
    robot.H_q = H_q_temp;
    robot.C_q = simplify(C_raw);
    
    % write function
    v_list = import_v_list(flagReduced);
    passing_var = {'q','L','m','I'};
    v_list_var = [v_list.q; v_list.L; v_list.m; v_list.I];
    if flagWriteFcn
        cd fcns
        write_fcn_m('fcn_H_q.m', passing_var, v_list_var, {robot.H_q, 'H'});
        cd ..
        disp('- writing completed')
    end
else    
    %% Reduced Model
    params_ = import_robot_params('symbolic', flagReduced);
    [H_raw, C_raw] = HandC(robot, robot.rq, robot.rqd);
    H_rq = simplify(H_raw);     % unreduced inertia
    C_rq = simplify(C_raw);
    
    % Constraint Jacobian & Nullspace Matrix
    R = params_.L(1);   % wheel radius
    K_con = [1 0 0 0; 1/R 0 0 0; -1/R 1 0 0; 0 0 1 0; 0 0 0 1];     % A_con = [-1/R 1 0 0 0];
    
    % Reduced System
    H_q_temp = simplify(expand(K_con'*H_raw*K_con));   % inertia matrix reduced
    C_q_temp = simplify(expand(K_con'*C_raw));   % inertia matrix reduced
    syms q1 qd1 qw qa1 qda1 qdw real
    H_q = subs(H_q_temp, [q1 + qw, qd1 + qdw], [qa1, qda1]);   % change of variable
    C_q = subs(C_q_temp, [q1 + qw, qd1 + qdw], [qa1, qda1]);   % change of variable
    
    % outputs
    robot.H_rq = H_rq;
    robot.C_rq = C_rq;
    robot.reduced.K_con = K_con;
    robot.reduced.H_q = H_q;
    robot.reduced.C_q = C_q;
        
    % write function
    v_list = import_v_list(flagReduced);
    passing_var = {'q','L','m','I'};
    v_list_var = [v_list.q; v_list.L; v_list.m; v_list.I];
    if flagWriteFcn
        cd fcns
        write_fcn_m('fcn_H_q_reduced.m', passing_var, v_list_var, {robot.reduced.H_q, 'H'});
        cd ..
        disp('- writing completed')
    end
    
end