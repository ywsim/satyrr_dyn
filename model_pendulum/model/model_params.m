function model_prms = model_params()

    % Dimensions (m)
    L_lower_leg = 42E-3;
    L_upper_leg = 45E-3; 
    L_torso     = 72E-3; 
    L_arm = 90E-3;      
    
    % COM distance from joint closer to the foot for bodies 1-4 (m)
    Lcom_lower_leg  = 0.5* L_lower_leg ;   
    Lcom_upper_leg  = 0.5* L_upper_leg ;    
    Lcom_torso      = 0.5*L_torso;          
    Lcom_arm = 0.5*L_arm;               
    
    % Position from world frame to contact points (Front and Back)
    rWF = 10E-3*[39; 0; -31];
    rWB = 10E-3*[-45; 0; -31];
    
    % Masses (kg)
    M_lower_leg = 8.99E-3; 
    M_upper_leg = 23E-3;    
    M_torso     = 250.48E-3;    
    M_arm = 53.42E-3;        
    
    % Stores body parameters
    model_prms.L    = [L_lower_leg; L_upper_leg; L_torso; L_arm];
    model_prms.L1   = L_lower_leg;
    model_prms.L2   = L_upper_leg;    
    model_prms.L3   = L_torso;
    model_prms.L4   = L_arm;
    
    model_prms.Lc   = [Lcom_lower_leg; Lcom_upper_leg; Lcom_torso; Lcom_arm];
    model_prms.Lc1  = Lcom_lower_leg;
    model_prms.Lc2  = Lcom_upper_leg;    
    model_prms.Lc3  = Lcom_torso;
    model_prms.Lc4  = Lcom_arm;
    
    model_prms.rWF = rWF;
    model_prms.rWB = rWB;

    model_prms.M    = [M_lower_leg; M_upper_leg; M_torso; M_arm];
    model_prms.M1   = M_lower_leg;
    model_prms.M2   = M_upper_leg;    
    model_prms.M3   = M_torso;
    model_prms.M4   = M_arm;

end