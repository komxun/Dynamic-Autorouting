%% Analyze Results — Timing & Trajectory Comparison
%  Run this after main.m finishes to compare against saved baselines.
%  Requires: timer_log, traj, rt, Object, delta_g, rtsim from workspace.
addpath('../src', '../plots', '../data')

%% Realtime timing analysis
s = load(fullfile('..', 'data', 'time_optim_journal_dyna_2.mat'));
figure('Name', 'Timing Comparison')
stem(timer_log(1:size(traj,2)), 'filled', 'LineWidth', 2, 'Marker', 'diamond')
hold on, grid on, grid minor
stem(s.timer(1:25), 'LineWidth', 1.5)
legend("IFDS Local Path", "Optimized IFDS Local Path")
xlabel("Elapsed Simulation Time (s)", 'FontSize', 20)
ylabel("Computed Time (s)", 'FontSize', 20)
set(gca, 'FontSize', 30, 'LineWidth', 1.5)

%% Trajectory comparison (optimized vs non-optimized)
allTraj = [traj{1:rt}];
tr = load(fullfile('..', 'data', 'allTraj_opt.mat'));

syms X Y Z
Gamma(X,Y,Z) = sym(0);
Gamma_star(X,Y,Z) = sym(0);

figure('Name', 'Trajectory Comparison')
subplot(1,2,1)
pltOpt = plot3(tr.allTraj(1,:), tr.allTraj(2,:), tr.allTraj(3,:), 'LineWidth', 2.5);
hold on, grid on, grid minor, axis equal
pltOg = plot3(allTraj(1,:), allTraj(2,:), allTraj(3,:), 'r--', 'LineWidth', 2.5);
PlotObject(Object, delta_g, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
camlight
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
set(gca, 'FontSize', 20, 'LineWidth', 1.5)
xlim([0 200]), ylim([-100 100]), zlim([0 100])

subplot(1,2,2)
plot3(tr.allTraj(1,:), tr.allTraj(2,:), tr.allTraj(3,:), 'LineWidth', 2.5);
hold on, grid on, grid minor, axis equal
plot3(allTraj(1,:), allTraj(2,:), allTraj(3,:), 'r--', 'LineWidth', 2.5);
PlotObject(Object, delta_g, rt, rtsim, X, Y, Z, Gamma, Gamma_star);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); camlight
view(0,90)
legend([pltOpt, pltOg], "Optimized Trajectory", "Non-optimized Trajectory")
set(gca, 'FontSize', 20, 'LineWidth', 1.5)
xlim([0 200]), ylim([-100 100])

%% Path length summary
traj_diff = diff(allTraj, 1, 2);
length_traj = sum(sqrt(sum(traj_diff.^2, 1)));
fprintf('Current trajectory length: %.2f m\n', length_traj);

traj_diff_opt = diff(tr.allTraj, 1, 2);
length_traj_opt = sum(sqrt(sum(traj_diff_opt.^2, 1)));
fprintf('Optimized trajectory length: %.2f m\n', length_traj_opt);
