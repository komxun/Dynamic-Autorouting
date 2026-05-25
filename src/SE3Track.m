function [pos_hist, vm, state, filters, timeSpent, logger] = ...
         SE3Track(Wi, Wf, state, filters, V_ref, dt_max, P, logger)
% SE3TRACK  SE(3) geometric tracker for a single IFDS path segment.
%
%   Ports the Lee et al. (2010/2011) geometric controller and full
%   rigid-body quadrotor dynamics from se3quad/matlab into the segmented
%   IFDS path-following framework (replaces CCA3D_2).
%
% Inputs :  Wi, Wf    3x1  segment endpoints (inertial frame)       [m]
%           state     struct with fields:
%                       p     3x1  inertial position               [m]
%                       v     3x1  inertial velocity               [m/s]
%                       R     3x3  body-to-inertial rotation matrix
%                       Omega 3x1  body-frame angular velocity     [rad/s]
%           filters   struct of DirtyDerivative handles (created by
%                     SE3Track_init); mutated in place.
%           V_ref     cruise reference speed along segment          [m/s]
%           dt_max    maximum segment duration (time budget)        [s]
%           P         UAV / controller parameter struct
%
% Outputs:  pos_hist  3xN  inertial positions visited this segment  [m]
%           vm        1xN  speed history  ||v||                     [m/s]
%           state     updated state struct (handed to next segment)
%           filters   updated filter handles
%           timeSpent simulated time elapsed in this segment         [s]
%
% Segment exit condition:
%   (a) UAV crosses the plane through Wf normal to the segment direction
%   (b) Reference point xd(t) has reached Wf  (|xd - Wi| >= |Wf - Wi|)
%   (c) dt_max elapsed (time budget for this main.m outer iteration)
%
% See also: se3_controller_step, hold_position

Ts = P.Ts;

%% Segment geometry
d_vec  = Wf - Wi;
L_seg  = norm(d_vec);
if L_seg < 1e-6
    pos_hist  = state.p;
    vm        = norm(state.v);
    timeSpent = 0;
    return
end
d_hat   = d_vec / L_seg;
psi_d   = atan2(d_hat(2), d_hat(1));
b1d     = [cos(psi_d); sin(psi_d); 0];

J  = diag([P.Jxx P.Jyy P.Jzz]);
m  = P.mass;  g = P.gravity;

%% Project current drone position onto segment
s0 = max(0, min(dot(state.p - Wi, d_hat), L_seg));

%% History buffers
N_max    = ceil(dt_max / Ts) + 1;
pos_hist = zeros(3, N_max);
vm       = zeros(1, N_max);
pos_hist(:,1) = state.p;
vm(1)         = norm(state.v);

%% Integration loop
t  = 0;
k  = 1;
while t < dt_max
    % Reference trajectory (linear along segment at V_ref)
    s       = V_ref * t + s0;
    xd      = Wi + min(s, L_seg) * d_hat;
    xd_1dot = V_ref * d_hat;
    xd_2dot = zeros(3,1);
    xd_3dot = zeros(3,1);
    xd_4dot = zeros(3,1);
    b1d_1dot = zeros(3,1);
    b1d_2dot = zeros(3,1);

    % Dirty-derivative filters
    v_1dot = filters.dv1dt.calculate(state.v);
    v_2dot = filters.dv2dt.calculate(v_1dot);

    % SE(3) controller
    [f, M_ctrl, Rc, Omegac, Omegac_1dot, Psi] = ...
        se3_controller_step(state, xd, xd_1dot, xd_2dot, xd_3dot, xd_4dot, ...
                            b1d, b1d_1dot, b1d_2dot, v_1dot, v_2dot, P);

    % Telemetry log
    if ~isempty(logger.t), t_global = logger.t(end) + Ts; else, t_global = 0; end
    deltaF = P.Mix * [f; M_ctrl];
    logger.t(:,end+1)      = t_global;
    logger.x(:,end+1)      = state.p;
    logger.xd(:,end+1)     = xd;
    logger.v(:,end+1)      = state.v;
    logger.vd(:,end+1)     = xd_1dot;
    logger.Omega(:,end+1)  = state.Omega;
    logger.Omegac(:,end+1) = Omegac;
    logger.Psi(:,end+1)    = Psi;
    logger.f(:,end+1)      = f;
    logger.M(:,end+1)      = M_ctrl;
    logger.deltaF(:,end+1) = deltaF;

    % RK4 integration
    state = rk4_step(state, f, M_ctrl, Ts, m, g, J);
    state.R = proj_SO3(state.R);

    % Bookkeeping
    t = t + Ts;
    k = k + 1;
    pos_hist(:,k) = state.p;
    vm(k)         = norm(state.v);

    % Segment-done check
    if dot(d_vec, state.p - Wf) >= 0
        break
    end
end

%% Trim history buffers
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
    if det(R) < 0
        R = U * diag([1, 1, -1]) * V';
    end
end
