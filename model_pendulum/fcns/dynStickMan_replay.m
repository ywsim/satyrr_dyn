function output = dynStickMan_replay(simTime, simOut, data)

% data
K = data.LQR_K;
u_0 = data.u_0;
s_0 = data.s_0;
mtot = data.m;
gvec = data.g;

% info
num_data_points = length(simTime);
num_states = (1/2)*length(s_0);
num_act = length(u_0);
num_F = 6;
num_con = 3;
num_com_coord = 3;
F_idx = [2 4 6];
f_D = data.disturbance_f;
t_applied_force_start = data.disturbance_time;
t_applied_force_duration = data.disturbance_duration; 

% initialize recorded data
q = zeros(num_data_points, num_states);
qd = zeros(num_data_points, num_states);
qdd = zeros(num_data_points, num_states);
p_com = zeros(num_data_points, num_com_coord);
u = zeros(num_data_points, num_act);
F = zeros(num_data_points, num_F);
F_con = zeros(num_data_points, num_con);
F_com = zeros(num_data_points, num_com_coord);

% replay simulation
for time_idx = 1:num_data_points
    
    % get t
    t = simTime(time_idx);
    
    % get y
    y = simOut(time_idx, :)';
    q(time_idx, :) = (s_0(1:4) + y(1:4))';
    qd(time_idx, :) = y(5:8)';
    
    % manipulator equations
    tau = -K*y + u_0;
    u(time_idx, :) = tau';
    dydt = [y(5:8);fcn_Z(s_0 + y) + fcn_Y(s_0 + y, tau)]';
    qdd(time_idx, :) = dydt(5:8);

    % centroidal dynamics
    F(time_idx, :) = (fcn_Acmm(s_0 + y)*dydt(5:8)' + fcn_Acmm_dot(s_0 + y)*dydt(1:4)' - mtot*gvec)';
    
    % center of mass
    p_com(time_idx, :) = fcn_cm(q(time_idx, :));
    
    % contact forces
    S_ext = fcn_S_ext(q(time_idx, :));   
    F_con(time_idx, :) = (S_ext\(F(time_idx, F_idx))')';
    
    % external force acting on COM
    if t >= t_applied_force_start && t <= (t_applied_force_start + t_applied_force_duration)
        F_com(time_idx, :) = f_D'; 
    else
        F_com(time_idx, :) = zeros(1, num_com_coord);  
    end    
    
end

% output
output.t = simTime;
output.q = q;
output.qd = qd;
output.qdd = qdd;
output.com = p_com;
output.u = u;
output.F = F;
output.F_con = F_con;
output.F_com = F_com;

end