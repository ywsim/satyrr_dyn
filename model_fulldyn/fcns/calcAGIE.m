function Z = calcAGIE(q_, L_, m_, I_, gear_, eff_, scenario, drive)
J_ = fcn_Jaco_Contact(q_, L_);

switch scenario
    case 'bwd'
        switch drive
            case 'ser'
                H = fcn_H_q_ser_bwd(q_, L_, m_, I_, gear_, eff_);
            case 'par'
                H = fcn_H_q_par_bwd(q_, L_, m_, I_, gear_, eff_);
            case 'dif'
                H = fcn_H_q_dif_bwd(q_, L_, m_, I_, gear_, eff_);
            otherwise
                disp('drivetrain must be ser/par/dif')
        end
        Z = inv( J_*(H\transpose(J_)));     % BGIE
        
    case 'fwd'
        switch drive
            case 'ser'
                H = fcn_H_q_ser_fwd(q_, L_, m_, I_, gear_, eff_);
                D = fcn_D_q_ser();
            case 'par'
                H = fcn_H_q_par_fwd(q_, L_, m_, I_, gear_, eff_);
                D = fcn_D_q_par();
            case 'dif'
                H = fcn_H_q_dif_fwd(q_, L_, m_, I_, gear_, eff_);
                D = fcn_D_q_dif();
            otherwise
                disp('drivetrain must be ser/par/dif')
        end
        E = fcn_E_q_fwd(eff_);
        G = fcn_G_q(gear_);
        tDG = transpose(D*G);
%         debug
%         J_'
%         (tDG\E)
%         (tDG\E)* tDG
%         ((tDG\E)* tDG* J_')
%         (H\((tDG\E)* tDG* J_'))
        Z = inv( J_*(H\((tDG\E)* tDG* J_')));     % FGIE
        
    case 'idl'
        switch drive
            case 'ser'
                H = fcn_H_q_ser_idl(q_, L_, m_, I_, gear_, eff_);
            case 'par'
                H = fcn_H_q_par_idl(q_, L_, m_, I_, gear_, eff_);
            case 'dif'
                H = fcn_H_q_dif_idl(q_, L_, m_, I_, gear_, eff_);
            otherwise
                disp('drivetrain must be ser/par/dif')
        end
        Z = inv( J_*(H\transpose(J_)));     % Task Space Inertia
        
    otherwise
        disp('scenario must be bwd/fwd/idl')
        
end



end
% H_rq_ser_bwd = fcn_H_rq_ser_bwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_par_bwd = fcn_H_rq_par_bwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_dif_bwd = fcn_H_rq_dif_bwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_ser_fwd = fcn_H_rq_ser_fwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_par_fwd = fcn_H_rq_par_fwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_dif_fwd = fcn_H_rq_dif_fwd(q_, L_, m_, I_, gear_, eff_);
% H_rq_ser_idl = fcn_H_rq_ser_idl(q_, L_, m_, I_, gear_, eff_);
% H_rq_par_idl = fcn_H_rq_par_idl(q_, L_, m_, I_, gear_, eff_);
% H_rq_dif_idl = fcn_H_rq_dif_idl(q_, L_, m_, I_, gear_, eff_);
% 
% out.Z_rq_ser_bwd = inv( J_*(H_rq_ser_bwd\transpose(J_)));
% out.Z_rq_par_bwd = inv( J_*(H_rq_par_bwd\transpose(J_)));
% out.Z_rq_dif_bwd = inv( J_*(H_rq_dif_bwd\transpose(J_)));
% out.Z_rq_ser_fwd = inv( J_*(H_rq_ser_fwd\transpose(J_)));
% out.Z_rq_par_fwd = inv( J_*(H_rq_par_fwd\transpose(J_)));
% out.Z_rq_dif_fwd = inv( J_*(H_rq_dif_fwd\transpose(J_)));
% out.Z_rq_ser_idl = inv( J_*(H_rq_ser_idl\transpose(J_)));
% out.Z_rq_par_idl = inv( J_*(H_rq_par_idl\transpose(J_)));
% out.Z_rq_dif_idl = inv( J_*(H_rq_dif_idl\transpose(J_)));
