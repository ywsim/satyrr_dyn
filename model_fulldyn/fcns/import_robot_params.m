function out = import_robot_params(flag_symbolic_value, flagReduced)
% flag_symbolic_value should be either 'symbolic' or 'value'
syms qx qdx qa1 qw qdw qda1 real
syms q qd I [3 1] real
syms L1 L2 L3x L3z real
syms R Iw real
syms mw m1 m2 m3 real

switch flag_symbolic_value
    case 'symbolic'
        if ~flagReduced         % unreduced model
            out.q = [qx; q1; q2; q3];   
            out.qd = [qdx; qd1; qd2; qd3];
        else        % reduced model
            out.q =  [qx; qa1; q2; q3];
            out.qd = [qdx; qda1; qd2; qd3];
            out.rq = [qx; qw; q1; q2; q3];
            out.rqd= [qdx; qdw; qd1; qd2; qd3];
        end
        
        out.L = [R; L1; L2; L3x; L3z];
        out.m = [mw; m1; m2; m3];
        out.I = [Iw; I1; I2; I3];
        
    case 'value'
        out.L = [0; 0.4; 0.4];
        out.m = [1; 2; 2];
        out.I = [0.5^2/6; 0.4^2/12; 0.4^2/12].*out.m;
    otherwise
        disp('Unknown method.')
end
