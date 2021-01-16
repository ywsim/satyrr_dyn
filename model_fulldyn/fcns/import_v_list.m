function v_list = import_v_list(flagReduce)
%% Refer to import_robot_params
%         syms qx qdx qw qdw real
%         syms q qd I [3 1] real
%         syms L1 L2 L3x L3z real
%         syms R Iw real
%         syms mw m1 m2 m3 real

%% List

if ~flagReduce      % unreduced model
    v_list.q = {
        'qx' 'q(1)';
        'q1' 'q(2)';
        'q2' 'q(3)';
        'q3' 'q(4)';
        };
    
    v_list.qd = {
        'qdx' 'q(1)';
        'qd1' 'q(2)';
        'qd2' 'q(3)';
        'qd3' 'q(4)';
        };
else                % reduced model
    v_list.rq = {
        'qx' 'q(1)';
        'qw' 'q(2)';
        'q1' 'q(3)';
        'q2' 'q(4)';
        'q3' 'q(5)';
        };
    
    v_list.rqd = {
        'qdx' 'qd(1)';
        'qdw' 'qd(2)';
        'qd1' 'qd(3)';
        'qd2' 'qd(4)';
        'qd3' 'qd(5)';
        };
    
    v_list.q = {
        'qx' 'q(1)';
        'qa1' 'q(2)';
        'q2' 'q(3)';
        'q3' 'q(4)';
        };
    
    v_list.qd = {
        'qdx' 'q(1)';
        'qda1' 'q(2)';
        'qd2' 'q(3)';
        'qd3' 'q(4)';
        };
end

v_list.L = {
    'R' 'L(1)';
    'L1' 'L(2)';
    'L2' 'L(3)';
    'L3x' 'L(4)';
    'L3z' 'L(5)';
    };

v_list.m = {
    'mw' 'm(1)';
    'm1' 'm(2)';
    'm2' 'm(3)';
    'm3' 'm(4)';
    };

v_list.I = {
    'Iw' 'I(1)';
    'I1' 'I(2)';
    'I2' 'I(3)';
    'I3' 'I(4)';
    };

% v_list.gear = {
%     'Irot1'  'gear(1)';
%     'Irot2'  'gear(2)';
%     'N1'     'gear(3)';
%     'N2'     'gear(4)';
%     };
%
% v_list.eff = {
%     'n_bwd1' 'eff(1)';
%     'n_bwd2' 'eff(2)';
%     'n_fwd1' 'eff(3)';
%     'n_fwd2' 'eff(4)';
%     };

end
