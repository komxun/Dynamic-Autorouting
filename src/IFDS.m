function [Paths, Object, totalLength, foundPath] = IFDS(rho0, sigma0, alpha_deg, loc_final, rt, Wp, Paths, Param, L, Object, weatherMat, dwdx, dwdy)
% IFDS  Interfered Fluid Dynamical System path planner.
%
%   [Paths, Object, totalLength, foundPath] = IFDS(rho0, sigma0, alpha_deg,
%       loc_final, rt, Wp, Paths, Param, L, Object, weatherMat, dwdx, dwdy)
%
%   Computes a collision-free path from the current waypoint to loc_final
%   using the IFDS velocity-field approach with weather-constraint coupling.
%
%   See also: create_scene, apply_weather, calc_ubar, create_shape

    % Unpack parameters
    simMode      = Param.simMode;
    scene        = Param.scene;
    sf           = Param.sf;
    targetThresh = Param.targetThresh;
    tsim         = Param.tsim;
    dt           = Param.dt;
    C            = Param.C;
    showDisp     = Param.showDisp;
    useOptimizer = Param.useOptimizer;
    delta_g      = Param.Rg;
    k            = Param.k;
    B_U          = Param.B_U;
    B_L          = Param.B_L;

    xd = loc_final(1);
    yd = loc_final(2);
    zd = loc_final(3);

    foundPath = 0;
    errFlag   = 0;

    % Iteration limit depends on sim mode
    if simMode == 1
        maxIter = tsim;
    else
        maxIter = 10000;
    end

    t = 1;
    while t <= maxIter
        Wp(:,t) = real(Wp(:,t));
        xx = Wp(1,t);
        yy = Wp(2,t);
        zz = Wp(3,t);

        % Scene geometry
        Object = create_scene(scene, Object, xx, yy, zz, rt, alpha_deg);

        % Check target reached
        if norm([xx yy zz] - [xd yd zd]) < targetThresh
            Wp = Wp(:,1:t);
            Paths{L,rt} = Wp;
            foundPath = 1;
            break
        end

        % Weather constraints
        if k ~= 0
            Object = apply_weather(Object, Param.numObj, k, B_L, B_U, xx, yy, weatherMat, dwdx, dwdy);
        end

        % Compute modulated velocity
        [UBar, rho0, sigma0, errFlag] = calc_ubar(xx, yy, zz, xd, yd, zd, ...
            Object, rho0, sigma0, useOptimizer, delta_g, C, sf, t);

        if errFlag == 1 && simMode == 1
            break
        end

        Wp(:,t+1) = Wp(:,t) + UBar * dt;
        t = t + 1;
    end

    % Finalise path
    Wp = Wp(:,1:t);
    Paths{L,rt} = Wp;

    if foundPath == 0 && errFlag == 0
        foundPath = 1;
    end

    %% Post-calculation: path length
    if foundPath == 1
        waypoints   = Paths{1,rt};
        differences = diff(waypoints, 1, 2);
        totalLength = sum(sqrt(sum(differences.^2, 1)));

        if showDisp
            fprintf('Total path length: %.2f m\n', totalLength);
            fprintf('Total flight time: %.2f s\n', totalLength/C);
        end
    else
        totalLength = 0;
    end
end
