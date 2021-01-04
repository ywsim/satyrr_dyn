function visualize_simulation(output)
% Simple visualization of simulation

time = output.t;
idx_plot = 1:5:length(time);
time = time(idx_plot, :);
q = output.q(idx_plot, :);
qd = output.qd(idx_plot, :);
u = output.u(idx_plot,:);
F_con = output.F_con(idx_plot,:);

figure()
subplot(2,2,1)
plot(time, q(:,1))
hold on
plot(time, q(:,2))
plot(time, q(:,3))
plot(time, q(:,4))
xlabel('time (s)')
title('Joint Angles (rad)')
legend('$q_{ankle}$', '$q_{knee}$', '$q_{hip}$', '$q_{shoulder}$', 'Interpreter','latex')
grid on

subplot(2,2,2)
plot(time, qd(:,1))
hold on
plot(time, qd(:,2))
plot(time, qd(:,3))
plot(time, qd(:,4))
xlabel('time (s)')
title('Joint Angular Velocities (rad/s)')
legend('$\dot{q}_{ankle}$', '$\dot{q}_{knee}$', '$\dot{q}_{hip}$', '$\dot{q}_{shoulder}$', 'Interpreter','latex')
grid on

subplot(2,2,3)
plot(time, u(:,1))
hold on
plot(time, u(:,2))
plot(time, u(:,3))
plot(time, u(:,4))
xlabel('time (s)')
title('Motor Torques (Nm)')
legend('$\tau_{ankle}$', '$\tau_{knee}$', '$\tau_{hip}$', '$\tau_{shoulder}$', 'Interpreter','latex')
grid on

subplot(2,2,4)
plot(time, F_con(:,1))
hold on
plot(time, F_con(:,2))
plot(time, F_con(:,3))
xlabel('time (s)')
title('Ground Reaction Forces (N)')
legend('$F_{B}^{x}$', '$F_{B}^{z}$', '$F_{F}^{z}$', 'Interpreter','latex')
grid on

end