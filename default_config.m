function cfg = default_config()
% DEFAULT_CONFIG  All tunable parameters for IFDS dynamic autorouting.
%
%   Edit this file to change simulation parameters.  Then run main.m.
%
%   cfg = default_config();

%% ======================== Simulation =====================================
cfg.tsim         = 100;       % [s]  max IFDS iterations per re-plan
cfg.dt           = 0.1;       % [s]  IFDS integration step
cfg.dt_traj      = 1;         % [s]  trajectory re-plan interval
cfg.rtsim        = 50;        % [-]  number of re-plan steps (rtsim = T / dt_traj)
cfg.simMode      = 2;         % 1: by time, 2: by target distance
cfg.targetThresh = 2;         % [m]  allowed error for final target distance

%% ======================== Scenario =======================================
cfg.scene        = 2;      % Scene number (see create_scene.m)
                              %   0) No object   1) 1 sphere   2) 2 objects
                              %   3) 3 objects   4) 3 complex  5) demo shapes
                              %   7) non-urban  12) urban      41/42/44) dynamic
                              %  100) GeoJSON buildings (oriented boxes)
cfg.multiTarget  = false;     % true: fly to multiple destinations

%% ---- GeoJSON scenario options (only used when cfg.scene == 100) ----------
cfg.geojson_file = fullfile('data', 'buildings.geojson');
cfg.geojson_alt  = 8;        % [m] cruise altitude (start/finish & target z)
cfg.geojson_p    = 4;        % super-ellipsoid exponent index (^8 -> sharp box)
cfg.geojson_pad  = 2;        % [m] lateral safety added to each box
cfg.geojson_corridor = 50;   % [m] keep buildings within this margin of the
                              %     start->finish line (only these run in IFDS)
cfg.geojson_mergeGap = 1.5;  % [m] merge footprints within this gap into one
                              %     box (0 = only touching; see load_buildings_geojson)

%% ======================== Safety ========================================
cfg.zFloor       = 2;        % [m] minimum AGL altitude; IFDS waypoints are
                              %     clamped so they never descend below this

%% ======================== IFDS Tuning ====================================
cfg.rho0         = 20.5;       % Repulsive parameter  (rho >= 0)
cfg.sigma0       = 20.5;      % Tangential parameter
cfg.sf           = uint8(0);  % Shape-following demand (1 = on, 0 = off)

%% ======================== Path Optimizer =================================
cfg.useOptimizer = 0;         % 0: Off,  1: Global optimized,  2: Local optimized
cfg.delta_g      = 2;        % [m]  minimum allowed gap distance

%% ======================== UAV ============================================
cfg.C            = 3;       % [m/s] UAV cruising speed

% Initial UAV state
cfg.x_i          = 0;         % [m]
cfg.y_i          = -20;       % [m]
cfg.z_i          = 5;         % [m]
cfg.psi_i        = 0;         % [rad] initial yaw
cfg.gamma_i      = 0;         % [rad] initial pitch

% IFDS path start (can differ from UAV position)
cfg.Xini         = 0;
cfg.Yini         = 0;
cfg.Zini         = 5;         % typically = z_i

% Target destination
cfg.Xfinal       = 200;       % [m]
cfg.Yfinal       = 0;         % [m]
cfg.Zfinal       = 50;        % [m]

%% ======================== SE(3) Controller ================================
% Ported from se3quad/matlab (Lee et al. 2010/2011, arXiv:1003.2005).
P.Ts        = 0.01;           % [s]       controller / integrator step
P.gravity   = 9.81;           % [m/s^2]
P.mass      = 4.34;           % [kg]
P.Jxx       = 0.0820;         % [kg m^2]
P.Jyy       = 0.0845;
P.Jzz       = 0.1377;
P.tau       = 0.05;           % dirty-derivative filter time constant

% Control gains
P.kx        = 4   * P.mass;
P.kv        = 5.6 * P.mass;
P.kR        = 8.81;
P.kOmega    = 2.54;

% Airframe geometry
P.d         = 0.315;          % [m]  CoM-to-rotor distance
P.c_tauf    = 8.004e-3;       % [m]  rotor drag/thrust ratio
P.Mix       = inv([1 1 1 1; 0 -P.d 0 P.d; ...
                   P.d 0 -P.d 0; -P.c_tauf P.c_tauf -P.c_tauf P.c_tauf]);

cfg.P = P;

%% ======================== Display ========================================
cfg.fontSize     = 20;
cfg.saveVid      = false;
cfg.animation    = false;     % true: animate Figure 69 frame-by-frame
cfg.showDisp     = true;

%% ============= GeoJSON scenario auto-configuration (scene 100) ============
% Loads building footprints, fits oriented parallelepipeds, places the
% start/finish just outside the building field, and disables the weather
% constraint (IFDS without weather coupling).
if cfg.scene == 100
    [B, dom] = load_buildings_geojson(cfg.geojson_file, ...
        struct('p', cfg.geojson_p, 'pad', cfg.geojson_pad, ...
               'mergeGap', cfg.geojson_mergeGap));
    cfg.buildings = B;
    cfg.domain    = dom;

    % --- IFDS without weather constraints ---
    cfg.k   = 0;            % no weather coupling
    cfg.env = "static";
    cfg.multiTarget = false;

    % --- Automated start / finish just outside the building area ---
    z = cfg.geojson_alt;
    cfg.x_i = dom.start(1);  cfg.y_i = dom.start(2);  cfg.z_i = z;
    cfg.Xini = dom.start(1); cfg.Yini = dom.start(2); cfg.Zini = z;
    cfg.Xfinal = dom.final(1); cfg.Yfinal = dom.final(2); cfg.Zfinal = z;

    % --- Trim buildings to the start->finish corridor (only these run IFDS) ---
    nBefore = numel(cfg.buildings);
    [cfg.buildings, ~] = trim_buildings_corridor(cfg.buildings, ...
        [cfg.x_i, cfg.y_i], [cfg.Xfinal, cfg.Yfinal], cfg.geojson_corridor);
    fprintf('Corridor trim: %d/%d buildings within %.0f m of start->finish line\n', ...
        numel(cfg.buildings), nBefore, cfg.geojson_corridor);

    % --- Sizing for the (larger) urban domain ---
    span = norm([cfg.Xfinal - cfg.Xini, cfg.Yfinal - cfg.Yini]);
    cfg.targetThresh = 5;
    cfg.rtsim = ceil(span / (cfg.C * cfg.dt_traj)) + 15;

    fprintf('GeoJSON scene: %d buildings | domain [%.0f %.0f] x [%.0f %.0f] m\n', ...
        numel(B), dom.xmin, dom.xmax, dom.ymin, dom.ymax);
    fprintf('Start (%.1f, %.1f, %.1f) -> Finish (%.1f, %.1f, %.1f)\n', ...
        cfg.x_i, cfg.y_i, cfg.z_i, cfg.Xfinal, cfg.Yfinal, cfg.Zfinal);
end

end
