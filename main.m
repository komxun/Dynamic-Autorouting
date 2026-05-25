%% IFDS Dynamic Autorouting
%  Komsun Tamanakijprasart (2023)
%
%  To configure: edit default_config.m, then run this script.
%  See docs/technical_report.md for algorithm details.
clc, clear, close all
addpath('src', 'plots', 'data')

%% 1. Load configuration
cfg = default_config();

%% 2. Setup simulation
[Param, Object, state, filters, logger, WMCell, dwdxCell, dwdyCell, ...
 weatherMat, weatherMatMod] = setup_simulation(cfg);

P        = cfg.P;
rho0     = cfg.rho0;
sigma0   = cfg.sigma0;
dt_traj  = cfg.dt_traj;
C        = cfg.C;
env      = cfg.env;
k        = cfg.k;
rtsim    = cfg.rtsim;
x_i = cfg.x_i;  y_i = cfg.y_i;  z_i = cfg.z_i;
Xini = cfg.Xini; Yini = cfg.Yini; Zini = cfg.Zini;
Xfinal = cfg.Xfinal; Yfinal = cfg.Yfinal; Zfinal = cfg.Zfinal;
targetThresh = cfg.targetThresh;
scene = cfg.scene;
useOptimizer = cfg.useOptimizer;
alpha_deg = 0;

% Destinations
if cfg.multiTarget
    destin = [200 0 20; 200 20 20; 200 -20 20; 200 20 30;
              200 -20 30; 200 0 30; 200 0 40; 200 20 40; 200 -20 40];
else
    destin = [Xfinal Yfinal Zfinal];
end
numLine = size(destin, 1);

% Pre-allocate
Wp    = zeros(3, cfg.tsim+1);
Paths = cell(numLine, rtsim);
traj  = cell(1, rtsim);
traj{1} = [x_i; y_i; z_i];
pos   = [x_i; y_i; z_i];
vhist = norm(state.v);
timer_log = zeros(1, rtsim);

fprintf('Generating paths for %d destination(s)...\n', numLine);

%% 3. Main IFDS path-planning loop
for rt = 1:rtsim
    tic

    % Check if target reached
    if norm([x_i y_i z_i] - [Xfinal Yfinal Zfinal]) < targetThresh
        fprintf('Target reached at t = %d s\n', rt);
        traj = traj(~cellfun('isempty', traj));
        break
    end

    % Set path start
    if scene == 41 || scene == 42 || (k ~= 0 && env == "dynamic") || scene == 44
        Wp(:,1) = [x_i; y_i; z_i];
    else
        Wp(:,1) = [Xini; Yini; Zini];
    end

    if isempty(traj{rt})
        traj{rt} = traj{rt-1}(:,end);
    end

    % Compute IFDS path for each destination line
    for L = 1:numLine
        loc_final = destin(L,:)';

        if useOptimizer == 1
            [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object, WMCell{rt}, dwdxCell{rt}, dwdyCell{rt});
        end

        if env == "dynamic"
            [Paths, Object, ~, foundPath] = IFDS(rho0, sigma0, alpha_deg, loc_final, rt, Wp, Paths, Param, L, Object, WMCell{rt}, dwdxCell{rt}, dwdyCell{rt});
        else
            [Paths, Object, ~, foundPath] = IFDS(rho0, sigma0, alpha_deg, loc_final, rt, Wp, Paths, Param, L, Object, WMCell{15}, dwdxCell{15}, dwdyCell{15});
        end
    end

    % Handle path-not-found: hover in place
    if foundPath ~= 1 || isempty(Paths{1,rt}) || size(Paths{1,rt},2) == 1
        fprintf('CAUTION: Path not found at t = %d s — holding position\n', rt);
        [hold_pos, hold_vm, state, filters, ~, logger] = ...
            hold_position(state, filters, dt_traj, P, logger);
        x_i = state.p(1);  y_i = state.p(2);  z_i = state.p(3);
        pos(:,end+1) = state.p;
        traj{rt}     = hold_pos;
        vhist(end+1) = hold_vm(end);
        continue
    end

    % SE(3) path following along IFDS waypoints
    trajectory = [x_i; y_i; z_i];
    dtcum = 0;

    for j = 1:size(Paths{1,rt},2)-1
        if dtcum >= dt_traj, break; end
        Wi = Paths{1,rt}(:,j);
        Wf = Paths{1,rt}(:,j+1);
        path_vect = Wf - Wi;

        % Skip waypoints already behind the UAV
        if dot(path_vect, [x_i; y_i; z_i] - Wf) >= 0
            continue
        end

        dt_budget = dt_traj - dtcum;
        [pos_seg, ~, state, filters, timeSpent, logger] = ...
            SE3Track(Wi, Wf, state, filters, C, dt_budget, P, logger);
        x_i = state.p(1);  y_i = state.p(2);  z_i = state.p(3);
        dtcum = dtcum + timeSpent;
        trajectory = [trajectory, pos_seg(:, 2:end)];
    end

    pos(:,end+1) = [x_i; y_i; z_i];
    traj{rt}     = trajectory;
    timer_log(rt) = toc;
    vhist(end+1) = norm(state.v);
    fprintf('rt=%d  computed in %.3f s\n', rt, timer_log(rt));
end

fprintf('Average compute time = %.4f s\n', mean(timer_log(timer_log ~= 0)));

%% 4. Plot results
% Workspace variables needed by plotting_everything.m
syms X Y Z Gamma(X,Y,Z) Gamma_star(X,Y,Z)
fontSize     = cfg.fontSize;
delta_g      = cfg.delta_g;
multiTarget  = cfg.multiTarget;
B_U          = cfg.B_U;

% --- Position & speed history ---
figure('Name', 'State History')
subplot(4,1,1), plot(pos(1,:), 'o-'), ylabel('X (m)'), grid on
subplot(4,1,2), plot(pos(2,:), 'o-'), ylabel('Y (m)'), grid on
subplot(4,1,3), plot(pos(3,:), 'o-'), ylabel('Z (m)'), grid on
subplot(4,1,4), plot(vhist, 'o-'), ylabel('Speed (m/s)'), xlabel('Step'), grid on

% --- SE(3) controller telemetry ---
se3_plot(logger, P);

% --- 3D trajectory overview (final frame) ---
figure('Name', 'IFDS 3D Result')
set(gcf, 'Position', get(0, 'Screensize'));
rt_plot = min(size(traj,2), rtsim);
rt = rt_plot;  % workspace variable used by plotting_everything.m

subplot(7,2,[1 3 5 7])
plotting_everything
if k ~= 0
    hold on, set(gca, 'YDir', 'normal'), colormap turbo
    if env == "dynamic"
        contourf(1:200,-100:99, weatherMatMod(:,:,rt_plot), 30, 'FaceAlpha',1,'LineStyle','none')
    else
        contourf(1:200,-100:99, weatherMatMod(:,:,15), 30, 'LineStyle','-')
    end
    hold off
end

subplot(7,2,[2 4 6 8])
plotting_everything
view(0,90), grid off

subplot(7,2,[9 11 13])
plotting_everything
view(90,0)

subplot(7,2,[10 12 14])
plotting_everything
view(0,0)

sgtitle(sprintf('IFDS  \\rho_0=%.2f  \\sigma_0=%.2f  scene=%d  t=%.0fs', ...
    cfg.rho0, cfg.sigma0, cfg.scene, rt_plot*dt_traj), 'FontSize', cfg.fontSize);
