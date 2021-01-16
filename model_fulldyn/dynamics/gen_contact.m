function robot = gen_contact(robot, flagWriteFcn)
params_ = import_robot_params('symbolic');

robot.contactBodyNum = 5;
robot.contactPos = [params_.L(3) 0 0];

jaco_xyz = contactJaco(robot, params_.rq, params_.rqd, robot.flagFloat, robot.contactBodyNum, robot.contactPos);
jaco_contact_redundant = simplify(jaco_xyz(1:2, :));
jaco_contact = jaco_contact_redundant(:, 1:robot.contactBodyNum);

robot.jaco_contact = jaco_contact;

v_list = import_v_list();
if flagWriteFcn
    cd fcns
    write_fcn_m('fcn_Jaco_Contact.m', {'q','L'}, [v_list.q; v_list.L], {robot.jaco_contact, 'jaco_contact'});
    cd ..
    disp('- writing completed')
end