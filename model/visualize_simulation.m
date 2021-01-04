function visualize_simulation(output)
% Simple visualization of simulation
time = output.t;

idx_arm_plot = 1:200:length(time);
time_arm = time(idx_arm_plot, :);

idx_plot = 1:5:length(time);
time = time(idx_plot, :);


% NOTE: joint positions and velocities and in DEGREE
q = rad2deg(output.q(idx_plot, :));
qd = rad2deg(output.qd(idx_plot, :));
q_cmd = rad2deg(output.q_cmd(idx_arm_plot , :));
qd_cmd = rad2deg(output.qd_cmd(idx_arm_plot , :));
u = output.u(idx_plot,:);
F_con = output.F_con(idx_plot,:);

n = 6; % # of DoFs
jointName = ["Ankle","Knee","Hip","Shoulder","Deltoid","Elbow"];
% define y-axis plotting limit so that all information are comfortably
% presented in the plots
jointPositionLimit = max(max(abs(q)))*[-1.2,1.2]; % joint position limit in deg
jointVelocityLimit = max(max(abs(qd)))*[-1.1,1.1]; % joint velocity limit in deg/s
jointTorqueLimit = max(max(abs(u)))*[-1.1,1.1]; % joint torque limit in N-m

% jointPositionLimit = 180*[-1,1]; % joint position limit in deg
% jointVelocityLimit = 200*[-1,1]; % joint velocity limit in deg/s
% jointTorqueLimit = 1*[-1,1]; % joint torque limit in N-m

% define figure position and size
set(0,'units','pixels'); % set screen size unit to pixel
screenSize = get(0,'screensize');
% ensure that the plot appears near the screen's center
figPositionAndSize = [0.25*screenSize(3),0.25*screenSize(4),800,600];


%% plot simulation results
plotJointInfo = figure(1);

% joint angle/vel/torques
for iDoF = 1:n
    % plot joint positions
    subplot(3,n,iDoF)
    plot(time,q(:,iDoF),'k-')
%     jointPositionLimit = max(abs(q(:,iDoF)))*[-1.2,1.2];
    ylim(jointPositionLimit)
    if iDoF < 2
        ylabel('Joint Position [deg]')
    end
    title(jointName(iDoF))
    grid on
    hold on
    
    % plot joint velocities
    subplot(3,n,n + iDoF)
    plot(time,qd(:,iDoF),'k-')
%     jointVelocityLimit = max(abs(qd(:,iDoF)))*[-1.2,1.2];
    ylim(jointVelocityLimit)
    if iDoF < 2
        ylabel('Joint Velocity [deg/s]')
    end
    grid on
    hold on
    
    % plot joint torques
    subplot(3,n,2*n + iDoF)
    plot(time,u(:,iDoF),'k-')
    ylim(jointTorqueLimit)
%     jointTorqueLimit = max(abs(u(:,iDoF)))*[-1.2,1.2];
    xlabel('Time [s]')
    if iDoF < 2
        ylabel('Joint Torque [N-m]')
    end
    grid on
end

% human motion command
for iDoF = 4:n
    % plot joint positions
    subplot(3,n,iDoF)
    plot(time_arm,q_cmd(:,iDoF-3),'r-')
    ylim(jointPositionLimit)
    if iDoF < 2
        ylabel('Joint Position [deg]')
    end
    title(jointName(iDoF))
    grid on
    if iDoF == 6
    legend('response', 'commanded')
    end
    % plot joint velocities
    subplot(3,n,n + iDoF)
    plot(time_arm,qd_cmd(:,iDoF-3),'r-')
    ylim(jointVelocityLimit)
    if iDoF < 2
        ylabel('Joint Velocity [deg/s]')
    end
    grid on
    if iDoF == 6
    legend('response', 'commanded')
    end
%     
%     % plot joint torques
%     subplot(3,n,2*n + iDoF)
%     plot(time,u(:,iDoF),'k-')
%     ylim(jointTorqueLimit)
%     xlabel('Time [s]')
%     if iDoF < 2
%         ylabel('Joint Torque [N-m]')
%     end
%     grid on
end
set(plotJointInfo,'Position',figPositionAndSize)

figPositionAndSize_GRF = [0.25*screenSize(3),0.25*screenSize(4),800,250];
plotGRFInfo = figure(2);
GRFName =  ["x-dir GRF, sole","z-dir GRF, hill","z-dir GRF, toe"]; % Fx; FzB; FzF;
for idx = 1:3
    subplot(1,3,idx)
    plot(time,F_con(:,idx),'k-')
    ylabel('Force, N')
    xlabel('time, s')
    if idx == 1
        ylim([-2, 2])
    else
        ylim([0, 2.5])
    end
    
    title(GRFName(idx))
    grid on
end
set(plotGRFInfo,'Position',figPositionAndSize_GRF)