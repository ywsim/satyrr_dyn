function robot = gen_satyrr_full(flagReduced)
%% Init
addpath(genpath(pwd))

robot.name = '2 DOF 2-Link Planar Monoped';
robot.base = 'floating';
robot.dimensions = 'planar';
robot.flagFloat = false;
robot.transmission = 'ser';
robot.flagReduced = flagReduced;

if ~flagReduced
    %% Unreduced Model
    robot.parent = [0 1 2 3 ];
    robot.nBase = 1;        % dof of the base
    robot.nLimbs = 3;       % dof of the links
    robot.nTransmission = 0;
    robot.nd = (robot.nBase + robot.nLimbs + robot.nTransmission);% number of DoF
    robot.NB = robot.nd;    % number of body
    
    % Import Params
    params_ = import_robot_params('symbolic', flagReduced);
    % out.L = [R; L1; L2; L3x; L3z];
    % out.m = [mw; m1; m2; m3];
    % out.I = [Iw; I1; I2; I3];
    
    % Rigid Body Params
    mass_ = [   params_.m(1);       % wheel mass
                params_.m(2);       % link mass
                params_.m(3);       % link mass
                params_.m(4)];      % link mass
    
    com_ = [    zeros(1,3);         % wheel
                0 0 params_.L(2)/2;
                0 0 params_.L(3)/2 
                params_.L(4) 0 params_.L(5)];
    
    iner_ = [   0 params_.I(1) 0;
                0 params_.I(2) 0;
                0 params_.I(3) 0;
                0 params_.I(4) 0];
    
    jtype_ = {'Px', 'Ry', 'Ry', 'Ry'};
    
    jpos_ = [   zeros(1,3);                 % world to the wheel
                zeros(1,3);                 % wheel to the 1st link
                0 0 params_.L(2) ;          % 1st link to the 2nd
                0 0 params_.L(3)];          % 2nd link to the 3rd
    
else
    %% Reduced Model
    robot.parent = [0 1 2 3 4 ];
    robot.nBase = 1;
    robot.nLimbs = 4;
    robot.nTransmission = 0;
    robot.nd = (robot.nBase + robot.nLimbs + robot.nTransmission);% number of DoF
    
    robot.NB = robot.nd;    % number of body
    
    % Import Params
    params_ = import_robot_params('symbolic', flagReduced);
    % out.L = [R; L1; L2; L3x; L3z];
    % out.m = [mw; m1; m2; m3];
    % out.I = [Iw; I1; I2; I3];
    
    % Rigid Body Params
    mass_ = [   0;                  % dummy joint
                params_.m(1);       % wheel mass
                params_.m(2);       % link mass
                params_.m(3);
                params_.m(4)];
    
    com_ = [    zeros(1,3);         % trivial
                zeros(1,3);         % wheel
                0 0 params_.L(2)/2;
                0 0 params_.L(3)/2 
                params_.L(4) 0 params_.L(5)];
    
    iner_ = [   zeros(1,3);
                0 params_.I(1) 0;
                0 params_.I(2) 0;
                0 params_.I(3) 0;
                0 params_.I(4) 0];
    
    jtype_ = {'Px', 'Ry', 'Ry', 'Ry', 'Ry'};
    
    jpos_ = [   zeros(1,3);         % world to dummy base
                zeros(1,3);         % dummy base to the wheel
                zeros(1,3);                 % wheel to the 1st link
                0 0 params_.L(2) ;          % 1st link to the 2nd
                0 0 params_.L(3)];          % 2nd link to the 3rd
end

%% Rigid Body mass, I, Xtree
for idx_body=1:robot.nd
    robot.jtype{idx_body} = jtype_{idx_body};          % joint type
    robot.com{idx_body}   = com_(idx_body,:);           % Com Position
    robot.mass{idx_body} = mass_(idx_body);                   % Body Mass
    robot.I{idx_body} = mcI(robot.mass{idx_body}, robot.com{idx_body}, diag(iner_(idx_body,:))); % Inertia
    robot.Xtree{idx_body} = plux(eye(3), jpos_(idx_body,:));   % Tree transform
end

%% outputs
robot.q = params_.q;
robot.qd = params_.qd;
if flagReduced
    robot.rq = params_.rq;
    robot.rqd = params_.rqd;
end


% % debug
% [H, C] = HandC(robot, robot.rq, robot.rqd);
% robot.H_ = simplify(H);
%
% syms Irot real
% robot.Hmot_ = robot.H_ - subs(robot.H_, Irot, 0)
% syms a
