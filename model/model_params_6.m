function model_prms = model_params_6()

    % Dimensions (m)
    L_lower_leg = 42E-3;
    L_upper_leg = 45E-3; 
    L_torso     = 72E-3; 
    L_arm_1 = 12E-3;
    L_arm_2 = 45E-3;
    L_arm_3 = 45E-3;
    
    % COM distance from joint closer to the foot for bodies (m)
    Lcom_lower_leg  = 0.5* L_lower_leg ;   
    Lcom_upper_leg  = 0.5* L_upper_leg ;    
    Lcom_torso      = 0.5*L_torso;          
    Lcom_arm_1 = 0.5*L_arm_1;
    Lcom_arm_2 = 0.5*L_arm_2;
    Lcom_arm_3 = 0.5*L_arm_3;
    
    % Position from world frame to contact points (Front and Back)
    rWF = 10E-3*[39; 0; -31];
    rWB = 10E-3*[-45; 0; -31];
    
    % Masses (kg)
    M_lower_leg = 8.99E-3; 
    M_upper_leg = 23E-3;    
    M_torso     = 250.48E-3;    
    M_arm_1 = 4.57E-3;        
    M_arm_2 = 12.05E-3;        
    M_arm_3 = 20E-3;      
    
    % Stores body parameters
    model_prms.L    = [L_lower_leg; L_upper_leg; L_torso; L_arm_1; L_arm_2; L_arm_3];
    model_prms.L1   = L_lower_leg;
    model_prms.L2   = L_upper_leg;    
    model_prms.L3   = L_torso;
    model_prms.L4   = L_arm_1;
    model_prms.L5   = L_arm_2;
    model_prms.L6   = L_arm_3;
    
    model_prms.Lc   = [Lcom_lower_leg; Lcom_upper_leg; Lcom_torso; Lcom_arm_1; Lcom_arm_2; Lcom_arm_3];
    model_prms.Lc1  = Lcom_lower_leg;
    model_prms.Lc2  = Lcom_upper_leg;    
    model_prms.Lc3  = Lcom_torso;
    model_prms.Lc4  = Lcom_arm_1;
    model_prms.Lc5  = Lcom_arm_2;
    model_prms.Lc6  = Lcom_arm_3;
    
    model_prms.rWF = rWF;
    model_prms.rWB = rWB;

    model_prms.M    = [M_lower_leg; M_upper_leg; M_torso; M_arm_1; M_arm_2; M_arm_3];
    model_prms.M1   = M_lower_leg;
    model_prms.M2   = M_upper_leg;    
    model_prms.M3   = M_torso;
    model_prms.M4   = M_arm_1;
    model_prms.M5   = M_arm_2;
    model_prms.M6   = M_arm_3;

end