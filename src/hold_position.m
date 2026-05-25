function [pos_hist, vm, state, filters, timeSpent, logger] = ...
         hold_position(state, filters, dt_hold, P, logger)
% HOLD_POSITION  Simulate SE(3) hover while IFDS has no path.
%
%   Commands xd = current position with zero reference velocity/accel.
%   The SE(3) geometric controller naturally regulates to hover: thrust
%   balances gravity, moments drive R to level (b1 toward psi = 0), and
%   velocity/angular-velocity errors damp to zero.
%
% Inputs :  state     current UAV state (p, v, R, Omega)
%           filters   DirtyDerivative filter handles (mutated in place)
%           dt_hold   duration of the hold                          [s]
%           P         UAV / controller parameters
%
% Outputs:  pos_hist  3xN  inertial position history (≈ constant)   [m]
%           vm        1xN  speed history (decays to 0)              [m/s]
%           state     updated state (should be near hover)
%           filters   updated filter handles
%           timeSpent simulated time elapsed                         [s]
%
% See also: se3_controller_step, SE3Track

Ts = P.Ts;
J  = diag([P.Jxx P.Jyy P.Jzz]);
m  = P.mass;  g = P.gravity;

% Hover target = current position, desired heading kept at zero (world +x)
xd  = state.p;
b1d = [1; 0; 0];

N_max    = ceil(dt_hold / Ts) + 1;
pos_hist = zeros(3, N_max);
vm       = zeros(1, N_max);
pos_hist(:,1) = state.p;
vm(1)         = norm(state.v);

t = 0; k = 1;
while t < dt_hold
    % Zero reference derivatives — hover
    xd_1dot  = zeros(3,1);
    xd_2dot  = zeros(3,1);
    xd_3dot  = zeros(3,1);
    xd_4dot  = zeros(3,1);
    b1d_1dot = zeros(3,1);
    b1d_2dot = zeros(3,1);

    % Dirty-derivative filters
    v_1dot = filters.dv1dt.calculate(state.v);
    v_2dot = filters.dv2dt.calculate(v_1dot);

    % SE(3) controller
    [f, M_ctrl, Rc, Omegac, ~, Psi] = ...
        se3_controller_step(state, xd, xd_1dot, xd_2dot, xd_3dot, xd_4dot, ...
                            b1d, b1d_1dot, b1d_2dot, v_1dot, v_2dot, P);

    % Telemetry log
    if ~isempty(logger.t), t_global = logger.t(end) + Ts; else, t_global = 0; end
    deltaF = P.Mix * [f; M_ctrl];
    logger.t(:,end+1)      = t_global;
    logger.x(:,end+1)      = state.p;
    logger.xd(:,end+1)     = xd;
    logger.v(:,end+1)      = state.v;
    logger.vd(:,end+1)     = zeros(3,1);
    logger.Omega(:,end+1)  = state.Omega;
    logger.Omegac(:,end+1) = Omegac;
    logger.Psi(:,end+1)    = Psi;
    logger.f(:,end+1)      = f;
    logger.M(:,end+1)      = M_ctrl;
    logger.deltaF(:,end+1) = deltaF;

    % RK4 integration
    state = rk4_step(state, f, M_ctrl, Ts, m, g, J);
    state.R = proj_SO3(state.R);

    t = t + Ts;  k = k + 1;
    pos_hist(:,k) = state.p;
    vm(k)         = norm(state.v);
end

pos_hist  = pos_hist(:, 1:k);
vm        = vm(1:k);
timeSpent = t;
end

%% =====================================================================
%% Local helpers (RK4 integration)
%% =====================================================================

function state = rk4_step(state, f, M, dt, m, g, J)
    k1 = deriv(state, f, M, m, g, J);
    s2 = add_state(state, k1, dt/2);  k2 = deriv(s2, f, M, m, g, J);
    s3 = add_state(state, k2, dt/2);  k3 = deriv(s3, f, M, m, g, J);
    s4 = add_state(state, k3, dt);    k4 = deriv(s4, f, M, m, g, J);
    kavg.p     = (k1.p     + 2*k2.p     + 2*k3.p     + k4.p)    /6;
    kavg.v     = (k1.v     + 2*k2.v     + 2*k3.v     + k4.v)    /6;
    kavg.R     = (k1.R     + 2*k2.R     + 2*k3.R     + k4.R)    /6;
    kavg.Omega = (k1.Omega + 2*k2.Omega + 2*k3.Omega + k4.Omega)/6;
    state      = add_state(state, kavg, dt);
end

function k = deriv(s, f, M, m, g, J)
    e3      = [0;0;1];
    k.p     = s.v;
    k.v     = g*e3 - (f/m)*(s.R*e3);
    k.R     = s.R * hat(s.Omega);
    k.Omega = J \ (M - cross(s.Omega, J*s.Omega));
end

function s_out = add_state(s, ds, dt)
    s_out.p     = s.p     + dt*ds.p;
    s_out.v     = s.v     + dt*ds.v;
    s_out.R     = s.R     + dt*ds.R;
    s_out.Omega = s.Omega + dt*ds.Omega;
end

function R = proj_SO3(R)
    [U, ~, V] = svd(R);
    R = U * V';
    if det(R) < 0, R = U * diag([1,1,-1]) * V'; end
end
