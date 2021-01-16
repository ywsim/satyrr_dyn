function data = dynStickMan_data(robot, ctrl, disturbance, coeff_fric)

% create data structure needed to run dynStickMan
data.nd = robot.nd;

% control data (from lowerbody_dynamics)
data.ctrl_type = ctrl.ctrl_type ;
data.LQR_K = ctrl.LQR.K;
data.LQR_Q = ctrl.LQR.Q;
data.LQR_R = ctrl.LQR.R;
data.u_0 = ctrl.u_0;        % 3 x 1
data.s_0 = ctrl.s_0;        % 6 x 1
data.u_stall = ctrl.u_stall;

% robot data (from full_dynamics) (total mass, contact points)
data.m = robot.mass_tot;
data.rWB = robot.contact.rWB;
data.rWF = robot.contact.rWF;

% gravity
data.g = get_gravity(robot);

% friction
data.coeff_fric = coeff_fric;

% disturbance
data.disturbance_f = disturbance.f;
data.disturbance_duration = disturbance.duration;
data.disturbance_time = disturbance.time;


end