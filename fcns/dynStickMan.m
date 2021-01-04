function dydt = dynStickMan(t, y, data)

%% data
K = data.LQR_K;
u_0 = data.u_0;
s_0 = data.s_0;
ctrl_type = data.ctrl_type;

%% disturbance - tau_D = J^T * f_D 
f_D = data.disturbance_f;
t_applied_force_start = data.disturbance_time;
t_applied_force_duration = data.disturbance_duration; 
if t >= t_applied_force_start && t <= (t_applied_force_start + t_applied_force_duration)
    tau_D = fcn_Jcm(s_0 + y)'*f_D; 
else
    tau_D = 0;  
end

%% qddot = -H\C + tau
% control
switch ctrl_type
    case 'LQR'
        du = -K*y;
    case 'QP'
        du = QP_stabilizing_controller(y, s_0, u_0, data);
end
% dynamics
dydt = [y(5:8);fcn_Z(s_0 + y) + fcn_Y(s_0 + y, u_0 + du + tau_D)];

end
