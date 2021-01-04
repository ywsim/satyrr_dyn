function [Z] = fcn_Z(s)

% Z = zeros(6,1);
Z = -fcn_H(s)\fcn_C(s);