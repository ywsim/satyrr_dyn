function out = inertia_reflected(q_, L_, m_, I_, H_mot)
J_ = fcn_Jaco_Contact(q_, L_);
H_ = fcn_H(q_, L_, m_, I_) + blkdiag(zeros(3), H_mot);
out = inv( J_*(H_\transpose(J_)));

end