function e_b = fcn_f2b_eff(e_f,N)
mu = 0.01;
G = 1/N;
sz_eff = length(e_f);
e_b = zeros(size(e_f));

for idx_eff = 1:sz_eff
    thres = (1-G^2)/2;
    e = e_f(idx_eff);
    if thres < e && e <=1
        e_b(idx_eff) = (2*e-1+G^2)/((1-G^2)*e+ 2*G^2);
    else
        e_b(idx_eff) = eps;
    end
end

end



