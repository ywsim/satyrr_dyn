function out = intercept_ellip(M, unit_vec, option)
u1 = unit_vec(1);
u2 = unit_vec(2);
if length(unit_vec)>2
    disp('unit vecor must be 2x1')
end
M_squared = M*M';
[V, D] = eig(M_squared);

% (c^2/a^2 + s^2/b^2)x^2 + 2cs(1/a^2-1/b^2)xy + (s^2/a^2 + c^2/b^2)y^2 = 1)
ang1 = atan2(V(2,1), V(1,1));
cc = cos(ang1)^2;
ss = 1 - cc;
aa = D(1,1);
bb = D(2,2);

A = cc/aa + ss/bb;
B = 2*cos(ang1)*sin(ang1)*(1/aa - 1/bb);
C = ss/aa + cc/bb;

t = sqrt(1/(A*u1^2 + B*u1*u2 + C*u2^2));

switch option
    case 'vector'
        out = t*unit_vec;
    case 'norm'
        out = t;
    otherwise
        disp('option should be vector or norm');
end


