function [Param, Object, state, filters, logger, WMCell, dwdxCell, dwdyCell, ...
         weatherMat, weatherMatMod] = setup_simulation(cfg)
% SETUP_SIMULATION  Build all data structures needed by the IFDS main loop.
%
%   [Param, Object, state, filters, logger, WMCell, dwdxCell, dwdyCell, ...
%    weatherMat, weatherMatMod] = setup_simulation(cfg)
%
%   Takes the cfg struct from default_config() and returns everything
%   main.m needs to run the simulation.

    P = cfg.P;

    %% Constraint matrix (weather data)
    matFile = fullfile('data', 'WeatherMat_321.mat');
    [weatherMat, weatherMatMod, WMCell, dwdxCell, dwdyCell] = ...
        initialize_constraint_matrix(matFile, cfg.B_L, cfg.B_U);

    %% Param table (passed to IFDS and helpers)
    Param.showDisp       = cfg.showDisp;
    Param.tsim           = cfg.tsim;
    Param.rtsim          = cfg.rtsim;
    Param.dt             = cfg.dt;
    Param.C              = cfg.C;
    Param.targetThresh   = cfg.targetThresh;
    Param.simMode        = cfg.simMode;
    Param.multiTarget    = cfg.multiTarget;
    Param.scene          = cfg.scene;
    Param.sf             = cfg.sf;
    Param.Rg             = cfg.delta_g;
    Param.rho0_initial   = cfg.rho0;
    Param.sigma0_initial = cfg.sigma0;
    Param.Xini           = cfg.x_i;
    Param.Yini           = cfg.y_i;
    Param.Zini           = cfg.z_i;
    Param.Xfinal         = cfg.Xfinal;
    Param.Yfinal         = cfg.Yfinal;
    Param.Zfinal         = cfg.Zfinal;
    Param.useOptimizer   = cfg.useOptimizer;
    Param.k              = cfg.k;
    Param.B_U            = cfg.B_U;
    Param.B_L            = cfg.B_L;

    %% Object structure pre-allocation
    switch cfg.scene
        case 0,    numObj = 1;
        case 1,    numObj = 1;
        case 2,    numObj = 2;
        case 3,    numObj = 3;
        case 4,    numObj = 3;
        case 5,    numObj = 3;
        case 7,    numObj = 7;
        case 12,   numObj = 12;
        case 41,   numObj = 3;
        case 42,   numObj = 4;
        case 44,   numObj = 7;
        case 69,   numObj = 4;
        case 6969, numObj = 3;
        otherwise, error('Unknown scene: %d', cfg.scene);
    end
    Param.numObj = numObj;

    Object(numObj) = struct('origin', zeros(cfg.rtsim, 3), ...
        'Gamma', 0, 'n', [], 't', [], ...
        'a', 0, 'b', 0, 'c', 0, ...
        'p', 0, 'q', 0, 'r', 0, 'Rstar', 0);

    %% SE(3) UAV state
    state.p     = [cfg.x_i; cfg.y_i; cfg.z_i];
    state.v     = zeros(3,1);
    state.R     = eye(3);
    state.Omega = zeros(3,1);

    %% Dirty-derivative filters
    filters.dv1dt = DirtyDerivative(1, P.tau,    P.Ts);
    filters.dv2dt = DirtyDerivative(2, P.tau*10, P.Ts);

    %% SE(3) telemetry logger
    logger.t      = [];
    logger.x      = [];
    logger.xd     = [];
    logger.v      = [];
    logger.vd     = [];
    logger.Omega  = [];
    logger.Omegac = [];
    logger.Psi    = [];
    logger.f      = [];
    logger.M      = [];
    logger.deltaF = [];

    %% Display summary
    fprintf('Scene %d — %d objects\n', cfg.scene, numObj);
    if cfg.sf == 0
        fprintf('Shape-following: Off\n');
    else
        fprintf('Shape-following: On\n');
    end
    switch cfg.useOptimizer
        case 0, fprintf('Path optimization: Off\n');
        case 1, fprintf('Path optimization: Global\n');
        case 2, fprintf('Path optimization: Local\n');
    end
end
