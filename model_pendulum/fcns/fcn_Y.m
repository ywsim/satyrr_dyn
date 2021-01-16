function [Y] = fcn_Y(s,u)

% Y = zeros(4,1);
Y = fcn_H(s)\u;
 