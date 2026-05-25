function se3_plot(L, P)
% SE3_PLOT  Batch visualisation of SE(3) controller telemetry.
%
%   Adapted from se3quad/matlab/plotter.m (Justin Thomas) for offline
%   plotting from a logger struct populated during simulation.
%
%   Three figures:
%     Figure 2 — Translational states  (x, y, z, v_x, v_y, v_z vs desired)
%     Figure 3 — Rotational states     (Omega_{x,y,z} vs Omegac, Psi)
%     Figure 4 — Actuators             (per-rotor force f_1..f_4 and
%                                       total thrust + moments)
%
% Inputs :  L   logger struct produced by SE3Track / hold_position
%           P   SE(3) parameter struct (for reference quantities)

if isempty(L.t)
    warning('se3_plot: logger is empty — nothing to plot.');
    return
end
t = L.t;  mg = P.mass * P.gravity;

%% ================ Figure 2 : Translational & Rotational States ========
figure(2); clf
sgtitle('Translational & Rotational States', 'fontsize', 20)

labels_x = {'x  [m]', 'y  [m]', 'z  [m]'};
labels_v = {'v_x  [m/s]', 'v_y  [m/s]', 'v_z  [m/s]'};
labels_O = {'\Omega_x  [rad/s]', '\Omega_y  [rad/s]', '\Omega_z  [rad/s]'};
for i = 1:3
    subplot(3, 3, 3*(i-1) + 1); hold on; grid on
    plot(t, L.x(i,:),  '-',  'LineWidth', 1.6)
    plot(t, L.xd(i,:), '--', 'LineWidth', 1.2)
    ylabel(labels_x{i})
    if i == 3, xlabel('t [s]'); end
    legend(["actual", "desired"])
    set(gca,"fontsize", 18, "linewidth", 1.5)

    subplot(3, 3, 3*(i-1) + 2); hold on; grid on
    plot(t, L.v(i,:),  '-',  'LineWidth', 1.6)
    plot(t, L.vd(i,:), '--', 'LineWidth', 1.2)
    ylabel(labels_v{i})
    legend(["actual", "commanded"])
    if i == 3, xlabel('t [s]'); end
    set(gca,"fontsize", 18, "linewidth", 1.5)

    subplot(3, 3, 3*(i-1) + 3); hold on; grid on
    plot(t, L.Omega(i,:),  '-',  'LineWidth', 1.6)
    plot(t, L.Omegac(i,:), '--', 'LineWidth', 1.2)
    ylabel(labels_O{i})
    legend(["actual", "commanded"])
    if i == 3, xlabel('t [s]'); end
    set(gca,"fontsize", 18, "linewidth", 1.5)
    ylim([-15,15])
end



%% ================ Figure 3 : Rotational States ======================
figure(3); clf
sgtitle('Rotational States (solid = actual, dashed = commanded)')

labels_O = {'\Omega_x  [rad/s]', '\Omega_y  [rad/s]', '\Omega_z  [rad/s]'};
for i = 1:3
    subplot(3, 2, 2*i - 1); hold on; grid on
    plot(t, L.Omega(i,:),  '-',  'LineWidth', 1.6)
    plot(t, L.Omegac(i,:), '--', 'LineWidth', 1.2)
    ylabel(labels_O{i})
    legend(["actual", "commanded"])
    if i == 3, xlabel('t [s]'); end
end

subplot(3, 2, [2 4 6]); hold on; grid on
plot(t, L.Psi, 'LineWidth', 1.6)
ylabel('\Psi  (SO(3) error)')
xlabel('t [s]')
title('Attitude error function  \Psi = \frac{1}{2} tr(I - R_c^T R)')

%% ================ Figure 4 : Actuators ==============================
figure(4); clf
sgtitle(['Actuators (dashed red: per-rotor hover thrust mg/4 = ', ...
         num2str(mg/4, '%.2f'), ' N)'], 'fontsize', 20)

for i = 1:4
    subplot(4, 2, 2*i - 1); hold on; grid on
    plot(t, L.deltaF(i,:), 'LineWidth', 1.4)
    yline(mg/4, 'r--', 'LineWidth', 1)
    ylabel(sprintf('f_%d  [N]', i))
    ylim([-200, 200])
    if i == 4, xlabel('t [s]'); end
    set(gca,"fontsize", 18, "linewidth", 1.5)
end

% Right column: total thrust + 3 moment axes
subplot(4, 2, 2);   hold on; grid on
plot(t, L.f, 'LineWidth', 1.4)
yline(mg, 'r--', 'LineWidth', 1)
ylabel('f_{total}  [N]')
set(gca,"fontsize", 18, "linewidth", 1.5)

labels_M = {'M_x  [N.m]', 'M_y  [N.m]', 'M_z  [N.m]'};
for i = 1:3
    subplot(4, 2, 2*(i+1)); hold on; grid on
    plot(t, L.M(i,:), 'LineWidth', 1.4)
    yline(0, 'k:', 'LineWidth', 1)
    ylabel(labels_M{i})
    ylim([-20, 20])
    if i == 3, xlabel('t [s]'); end
    set(gca,"fontsize", 18, "linewidth", 1.5)
end

%% ================ Figure 5 : Position error + thrust magnitude =====
figure(5); clf
sgtitle('Tracking-error norms', 'fontsize', 20)

pos_err = sqrt(sum((L.x - L.xd).^2, 1));
vel_err = sqrt(sum((L.v - L.vd).^2, 1));

subplot(3, 1, 1); plot(t, pos_err, 'LineWidth', 1.4); grid on
ylabel('||x - x_d||  [m]')
title('Position error')
set(gca,"fontsize", 18, "linewidth", 1.5)

subplot(3, 1, 2); plot(t, vel_err, 'LineWidth', 1.4); grid on
ylabel('||v - v_d||  [m/s]')
title('Velocity error')
set(gca,"fontsize", 18, "linewidth", 1.5)

subplot(3, 1, 3); plot(t, L.Psi, 'LineWidth', 1.4); grid on
ylabel('\Psi')
xlabel('t [s]')
title('Attitude error  \Psi')
set(gca,"fontsize", 18, "linewidth", 1.5)

end
