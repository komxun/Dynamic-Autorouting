function [UBar, rho0, sigma0, errFlag] = calc_ubar(X, Y, Z, xd, yd, zd, Obj, rho0, sigma0, useOptimizer, delta_g, C, sf, time)
% CALC_UBAR  Compute the IFDS modulated velocity vector.
%
%   Evaluates the perturbation (modular) matrix M for each obstacle and
%   returns the weighted modified velocity UBar = M_total * u.
%
%   Inputs:
%     X, Y, Z       - current UAV position
%     xd, yd, zd    - target destination
%     Obj            - array of Object structs (Gamma, n, t, etc.)
%     rho0, sigma0   - IFDS repulsive / tangential parameters
%     useOptimizer   - 0: off, 2: local optimized
%     delta_g        - [m] minimum gap distance
%     C              - [m/s] cruising speed
%     sf             - shape-following flag
%     time           - current iteration index
%
%   Outputs:
%     UBar           - 3x1 modified velocity
%     rho0, sigma0   - (possibly updated by local optimizer)
%     errFlag        - 1 if error condition encountered

    dist = sqrt((X - xd)^2 + (Y - yd)^2 + (Z - zd)^2);
    u = -[C*(X - xd)/dist; C*(Y - yd)/dist; C*(Z - zd)/dist];

    numObj  = size(Obj, 2);
    Mm      = zeros(3);
    sum_w   = 0;
    errFlag = 0;

    for j = 1:numObj
        Gamma = Obj(j).Gamma;
        n     = Obj(j).n;
        t     = Obj(j).t;

        % Object distance from UAV
        x0 = Obj(j).origin(1);
        y0 = Obj(j).origin(2);
        z0 = Obj(j).origin(3);
        dist_obj = sqrt((X - x0)^2 + (Y - y0)^2 + (Z - z0)^2);

        % Modular matrix (perturbation matrix)
        ntu = n' * u;
        if ntu < 0 || sf == 1
            % Local optimizer
            if useOptimizer == 2
                if mod(time, 5) == 0
                    [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0);
                end
            end

            % Gap constraint (safeguard)
            Rstar     = Obj(j).Rstar;
            rho0_star = log(abs(Gamma)) / (log(abs(Gamma - ((Rstar + delta_g)/Rstar)^2 + 1))) * rho0;
            rho       = rho0_star * exp(1 - 1/(dist_obj * dist));
            sigma     = sigma0 * exp(1 - 1/(dist_obj * dist));

            M = eye(3) - n*n' / (abs(Gamma)^(1/rho) * (n')*n) ...
                + t*n' / (abs(Gamma)^(1/sigma) * norm(t) * norm(n));

        elseif ntu >= 0 && sf == 0
            M = eye(3);
        else
            errFlag = 1;
            UBar = u;
            return
        end

        % Weight
        w = 1;
        for i = 1:numObj
            if i ~= j
                w = w * (Obj(i).Gamma - 1) / ((Obj(j).Gamma - 1) + (Obj(i).Gamma - 1));
            end
        end
        sum_w = sum_w + w;

        Obj(j).dist = dist_obj;
        Obj(j).M    = M;
        Obj(j).w    = w;
    end

    for j = 1:numObj
        Obj(j).w_tilde = Obj(j).w / sum_w;
        Mm = Mm + Obj(j).w_tilde * Obj(j).M;
    end

    UBar = Mm * u;
end

%% ========================================================================
function [rho0, sigma0] = path_opt2(Gamma, n, t, u, dist, dist_obj, rho0, sigma0)
% PATH_OPT2  Local fmincon optimization of rho0, sigma0.

    xg = [rho0; sigma0];

    problem.objective = @(x) norm_ubar(x(1), x(2), Gamma, n, t, u, dist, dist_obj);
    problem.x0        = xg;
    problem.Aineq     = [];
    problem.bineq     = [];
    problem.Aeq       = [];
    problem.beq       = [];
    problem.lb        = [0.05; 0];
    problem.ub        = [2; 1];
    problem.nonlcon   = [];
    problem.solver    = 'fmincon';
    problem.options   = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', 'Display', 'off');

    [xOpt, ~, ~, ~] = fmincon(problem);
    rho0   = xOpt(1);
    sigma0 = xOpt(2);
end
