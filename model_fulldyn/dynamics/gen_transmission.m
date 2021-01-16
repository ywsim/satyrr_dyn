function robot = gen_transmission(robot, flagWriteFcn)
params_ = import_robot_params('symbolic');
N = params_.N;

% Reduction Matrix (Generic)
tran.G = diag(1./N);

% Drivetrain Matrix (robot-specific, you need to write)
tran.D.ser = eye(robot.nTransmission);
tran.D.par = [1 0;-1 1];
tran.D.dif = .5*[1 1;-1 1];

robot.tran = tran;


% write function
v_list = import_v_list();
if flagWriteFcn
    cd fcns
    write_fcn_m('fcn_G_sub.m', {'gear'}, v_list.gear, {tran.G, 'G'});
    write_fcn_m('fcn_D_sub_ser.m', {}, [], {tran.D.ser, 'D'});
    write_fcn_m('fcn_D_sub_par.m', {}, [], {tran.D.par, 'D'});
    write_fcn_m('fcn_D_sub_dif.m', {}, [], {tran.D.dif, 'D'});
    disp('- writing completed')
    cd ..
end



end