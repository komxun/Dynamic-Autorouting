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
cfg.scene        = 3;        % Scene number (see create_scene.m)
                              %   0) No object   1) 1 sphere   2) 2 objects
                              %   3) 3 objects   4) 3 complex  5) demo shapes
                              %   7) non-urban  12) urban      41/42/44) dynamic
cfg.env          = "static"; % "static" or "dynamic" environmental constraint
cfg.multiTarget  = false;     % true: fly to multiple destinations

%% ======================== IFDS Tuning ====================================
cfg.rho0         = 2.5;       % Repulsive parameter  (rho >= 0)
cfg.sigma0       = 0.01;      % Tangential parameter
cfg.sf           = uint8(0);  % Shape-following demand (1 = on, 0 = off)

%% ======================== Constraint Matrix ==============================
cfg.k            = 0.5;       % Weather coupling gain (0 = no weather effect)
cfg.B_U          = 0.9;       % Upper bound  [B_L < B_U <= 1]
cfg.B_L          = 0;         % Lower bound  [0 <= B_L < B_U]

%% ======================== Path Optimizer =================================
cfg.useOptimizer = 0;         % 0: Off,  1: Global optimized,  2: Local optimized
cfg.delta_g      = 10;        % [m]  minimum allowed gap distance

%% ======================== UAV ============================================
cfg.C            = 9.5;       % [m/s] UAV cruising speed

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
cfg.animation    = true;     % true: animate Figure 69 frame-by-frame
cfg.showDisp     = true;

end
