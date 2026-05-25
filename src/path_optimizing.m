function [rho0, sigma0] = path_optimizing(loc_final, rt, Wp, Paths, Param, Object, weatherMat, dwdx, dwdy)
% PATH_OPTIMIZING  Global path-length optimization of rho0, sigma0.
%
%   Uses fmincon to find (rho0, sigma0) that minimises the total IFDS path
%   length to loc_final.  Called when cfg.useOptimizer == 1.

    xg = [Param.rho0_initial; Param.sigma0_initial];

    problem.objective = @(x) PathDistObjective(x(1), x(2));
    problem.x0        = xg;
    problem.Aineq     = [];
    problem.bineq     = [];
    problem.Aeq       = [];
    problem.beq       = [];
    problem.lb        = [0.05; 0];
    problem.ub        = [2.5;  2];
    problem.nonlcon   = [];
    problem.solver    = 'fmincon';
    problem.options   = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', 'Display', 'off', 'MaxIterations', 1);

    [xOpt, ~, ~, ~] = fmincon(problem);
    rho0   = xOpt(1);
    sigma0 = xOpt(2);

    function totalLength = PathDistObjective(rho0_in, sigma0_in)
        ParamLocal = Param;
        ParamLocal.showDisp     = 0;
        ParamLocal.useOptimizer = 0;
        [~, ~, totalLength, ~] = ...
            IFDS(rho0_in, sigma0_in, 0, loc_final, rt, Wp, Paths, ParamLocal, 1, Object, weatherMat, dwdx, dwdy);
    end
end
