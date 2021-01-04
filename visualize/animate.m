function animate(output,data)
%% Visualize Planar MINI as a Planar 4-DoF Manipulator
% Unit: mm-g-s-rad
% Point Definition:
% O -> ankle (inertial frame). K -> knee. H -> hip. S -> shoulder. A -> hand
% qNominal = [-pi/3,-pi/3,pi/6,0];
tic
% t = output.t; % simulation time
% q = output.q; % joint positions
% pCoM = 1000*output.p_com; % robot whole-body CoM position
% FContact = output.F_con; % foot-ground contact forces in [F_x_Back; F_z_Back; F_z_Front]
% FPerturb = output.F_com;
dt = 2e-2;
t = 0:dt:data.simTimeEnd;
q = interp1(output.t,output.q,t);
qcmd = interp1(output.t,output.q_cmd,t);
pCoM = 1000*interp1(output.t,output.p_com,t);
FContact = interp1(output.t,output.F_con,t);
% FPerturb = interp1(output.t,output.F_com,t);

[N,n] = size(q);
L_lower_leg = 42;
L_upper_leg = 45;
L_torso     = 72;
L_shoulder = 12;
L_upper_arm = 45;
L_lower_arm = 45;
LR = [L_lower_leg,L_upper_leg,L_torso,L_shoulder,L_upper_arm,L_lower_arm]; % robot link lengths
% footPosition = [[39; 0; -31],[-45; 0; -31]];
footPosition = 100*[data.rWF,data.rWB]; % robot foot-ground contact positions

xmax = 150;
ymax = 100;
zmax = 250;
plotWidth = 2;
scatterSize = 40;
txtRobotJoint = ["Ankle (Inertial Frame)","Knee","Hip","Shoulder","Deltoid","Elbow","Hand"];
txtRobotFootGroundContact = ["Toe","Heel"];

T = zeros(n,4,4);
Tcmd = zeros(n,4,4);
fig = figure(99);
video = VideoWriter('Animation','MPEG-4');
video.FrameRate = 60;
open(video)
set(fig,'doublebuffer','on');
for iTime = 1:N
    for iDoF = 1:n
        qThis = q(iTime,iDoF);
        
        if iDoF <= 4 % lower body
            rotationy = [cos(qThis),0,sin(qThis); 0,1,0; -sin(qThis),0,cos(qThis)];
            T(iDoF,:,:) = [rotationy,[LR(iDoF)*cos(qThis); 0; -LR(iDoF)*sin(qThis)]; [0,0,0,1]];
        else
            rotationz = [cos(qThis),-sin(qThis),0; sin(qThis),cos(qThis),0; 0,0,1];
            T(iDoF,:,:) = [rotationz,[LR(iDoF)*cos(qThis); LR(iDoF)*sin(qThis); 0]; [0,0,0,1]];
        end
        
        if iDoF > 3 % commanded arm
            iDoFcmd = iDoF - 3;
            qcmdThis = qcmd(iTime,iDoFcmd);
            if iDoFcmd < 2
                rotationycmd = [cos(qcmdThis),0,sin(qcmdThis); 0,1,0; -sin(qcmdThis),0,cos(qcmdThis)];
                Tcmd(iDoFcmd,:,:) = [rotationycmd,[LR(iDoF)*cos(qcmdThis); 0; -LR(iDoF)*sin(qcmdThis)]; [0,0,0,1]];
            else
                rotationzcmd = [cos(qcmdThis),-sin(qcmdThis),0; sin(qcmdThis),cos(qcmdThis),0; 0,0,1];
                Tcmd(iDoFcmd,:,:) = [rotationzcmd,[LR(iDoF)*cos(qcmdThis); LR(iDoF)*sin(qcmdThis); 0]; [0,0,0,1]];
            end
        end
    end
    TROK = squeeze(T(1,:,:)); % knee pose in {O}
    TROH = TROK*squeeze(T(2,:,:)); % hip pose in {O}
    TROS = TROH*squeeze(T(3,:,:)); % shoulder pose in {O}
    TROD = TROS*squeeze(T(4,:,:)); % deltoid pose in {O}
    TROE = TROD*squeeze(T(5,:,:)); % elbow pose in {O}
    TROA = TROE*squeeze(T(6,:,:)); % hand pose in {O}
    
    TRODcmd = TROS*squeeze(Tcmd(1,:,:)); % commanded deltoid position in {O}
    TROEcmd = TRODcmd*squeeze(Tcmd(2,:,:)); % commanded elbow position in {O}
    TROAcmd = TROEcmd*squeeze(Tcmd(3,:,:)); % commanded hand position in {O}
    
    pR = [zeros(3,1),getp(TROK),getp(TROH),getp(TROS),getp(TROD),getp(TROE),getp(TROA)];
    pRcmd = [getp(TROS),getp(TRODcmd),getp(TROEcmd),getp(TROAcmd)];
    
    pRFoot = [pR(:,1),footPosition,pR(:,1)]; % triangular foot vertices
    [groundx,groundy] = meshgrid(xmax*[-1,1],ymax*[-1,1]);
    
    % animate
    % draw links and joints
    surf(groundx,groundy,footPosition(3,1)*ones(size(groundx)),'facealpha',0.2,'facecolor','g') % draw ground
    hold on
    plotCmdArm = plot3(pRcmd(1,:),pRcmd(2,:),pRcmd(3,:),'r--','linewidth',plotWidth); % draw commanded robot arm link
    scatter3(pRcmd(1,:),pRcmd(2,:),pRcmd(3,:),scatterSize,'r','filled') % draw commanded robot arm joint
    plotLinks = plot3(pR(1,:),pR(2,:),pR(3,:),'k-','linewidth',plotWidth); % draw robot links
    plot3(pRFoot(1,:),pRFoot(2,:),pRFoot(3,:),'k-','linewidth',plotWidth) % draw robot triangular foot
    plot3([0,pCoM(iTime,1)],[0,pCoM(iTime,2)],[0,pCoM(iTime,3)],'m--','linewidth',plotWidth) % draw robot equivalent pendulum
    plotCoM = scatter3(pCoM(iTime,1),pCoM(iTime,2),pCoM(iTime,3),2*scatterSize,'m','filled'); % draw robot whole-body CoM position
    scatter3(pRFoot(1,1:end - 1),pRFoot(2,1:end - 1),pRFoot(3,1:end - 1),scatterSize,'k','filled') % draw inertial frame and foot-ground contact points
    plotJoints = scatter3(pR(1,2:end - 1),pR(2,2:end - 1),pR(3,2:end - 1),scatterSize,'b','filled'); % draw robot joints
    plotEndEffector = scatter3(pR(1,end),pR(2,end),pR(3,end),scatterSize,'r','filled'); % draw end-effector
    
    % draw foot-ground contact forces
    quiver3(footPosition(1,1),footPosition(2,1),footPosition(3,1),0,0,FContact(iTime,3),50,'linewidth',plotWidth,'color','r')
    quiver3(footPosition(1,2),footPosition(2,2),footPosition(3,2),0,0,FContact(iTime,2),50,'linewidth',plotWidth,'color','r')
    plotContactForce = quiver3(mean(footPosition(1,:)),mean(footPosition(2,:)),0.5*mean(footPosition(3,:)),FContact(iTime,1),0,0,400,'linewidth',plotWidth,'color','r');
    
    % draw text
    text(mean(pRFoot(1,1:end - 1)) - 14,mean(pRFoot(2,1:end - 1)),mean(pRFoot(3,1:end - 1) + 4),"Foot")
    yTextContactOffset = [-10,-10];
    for iFootGroundContact = 1:2
        text(footPosition(1,iFootGroundContact),footPosition(2,iFootGroundContact) + yTextContactOffset(iFootGroundContact),footPosition(3,iFootGroundContact),txtRobotFootGroundContact(iFootGroundContact))
    end
    hold off
    axis equal
    xlim(xmax*[-0.5,1])
    ylim(ymax*[-1,1])
    zlim(zmax*[-0.2,1])
    xlabel('x_R [mm]')
    ylabel('y_R [mm]')
    zlabel('z_R [mm]')
    title(strcat("Time = ",num2str(round(t(iTime),3))," s"))
    legend([plotLinks,plotJoints,plotEndEffector,plotCoM,plotCmdArm,plotContactForce],{'Link','Joint','End-Effector','Whole-Body CoM','Commanded Arm from Human Motion Data','Ground Reaction Force'},'location','north')
    grid on
    view(-45,20)
    set(fig,'Position',[500,100,800,800])
    
    pause(1e-3)
    frame = getframe(fig);
    drawnow;
    writeVideo(video,frame);
    
    percentFinished = round(100*iTime/N,2);
    if mod(percentFinished,10) <= 1
        disp(strcat("Animation Progress = ",num2str(percentFinished),"%"))
    end
end
close(video);
toc
end

function p = getp(T)
    p = T(1:3,end);
end

