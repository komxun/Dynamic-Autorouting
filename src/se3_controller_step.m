function [f, M, Rc, Omegac, Omegac_1dot, Psi] = ...
    se3_controller_step(state, xd, xd_1dot, xd_2dot, xd_3dot, xd_4dot, ...
                        b1d, b1d_1dot, b1d_2dot, v_1dot, v_2dot, P)
% SE3_CONTROLLER_STEP  One step of the Lee (2010/2011) geometric controller.
%
%   Computes thrust f and moment M from current state and reference trajectory.
%   Shared by SE3Track and hold_position.
%
%   Inputs:
%     state     - struct with fields: p, v, R, Omega
%     xd..xd_4dot - position reference and its derivatives
%     b1d, b1d_1dot, b1d_2dot - desired heading and derivatives
%     v_1dot, v_2dot - filtered acceleration/jerk of actual velocity
%     P         - parameter struct (mass, gravity, kx, kv, kR, kOmega, Jxx, Jyy, Jzz)
%
%   Outputs:
%     f         - scalar thrust [N]
%     M         - 3x1 moment vector [N.m]
%     Rc        - 3x3 desired rotation
%     Omegac    - 3x1 commanded body-rate
%     Omegac_1dot - 3x1 commanded body-rate derivative
%     Psi       - SO(3) geodesic error

    e3 = [0; 0; 1];
    J  = diag([P.Jxx P.Jyy P.Jzz]);
    m  = P.mass;
    g  = P.gravity;

    ex = state.p - xd;
    ev = state.v - xd_1dot;
    ea = v_1dot  - xd_2dot;
    ej = v_2dot  - xd_3dot;

    % Thrust direction / magnitude (Lee Eq. 19)
    A  = -P.kx*ex - P.kv*ev - m*g*e3 + m*xd_2dot;
    nA = norm(A);
    if nA < 1e-6, nA = 1e-6; end
    f  = dot(-A, state.R*e3);

    % Desired attitude Rc
    b3c = -A / nA;
    Cv  = cross(b3c, b1d);
    nC  = norm(Cv);
    if nC < 1e-6, Cv = [0;1;0]; nC = 1; end
    b1c = -(1/nC) * cross(b3c, Cv);
    b2c =  Cv / nC;
    Rc  = [b1c, b2c, b3c];

    % Time derivatives of Rc (Lee 2011 Appendix F)
    A_1dot   = -P.kx*ev - P.kv*ea + m*xd_3dot;
    b3c_1dot = -A_1dot/nA + (dot(A, A_1dot)/nA^3)*A;
    C_1dot   = cross(b3c_1dot, b1d) + cross(b3c, b1d_1dot);
    b2c_1dot = C_1dot/nC - (dot(Cv, C_1dot)/nC^3)*Cv;
    b1c_1dot = cross(b2c_1dot, b3c) + cross(b2c, b3c_1dot);

    A_2dot   = -P.kx*ea - P.kv*ej + m*xd_4dot;
    b3c_2dot = -A_2dot/nA + (2/nA^3)*dot(A, A_1dot)*A_1dot ...
             + ((norm(A_1dot)^2 + dot(A, A_2dot))/nA^3)*A  ...
             - (3/nA^5)*(dot(A, A_1dot)^2)*A;
    C_2dot   = cross(b3c_2dot, b1d) + cross(b3c, b1d_2dot) ...
             + 2*cross(b3c_1dot, b1d_1dot);
    b2c_2dot = C_2dot/nC - (2/nC^3)*dot(Cv, C_1dot)*C_1dot  ...
             - ((norm(C_1dot)^2 + dot(Cv, C_2dot))/nC^3)*Cv ...
             + (3/nC^5)*(dot(Cv, C_1dot)^2)*Cv;
    b1c_2dot = cross(b2c_2dot, b3c) + cross(b2c, b3c_2dot)  ...
             + 2*cross(b2c_1dot, b3c_1dot);

    Rc_1dot     = [b1c_1dot, b2c_1dot, b3c_1dot];
    Rc_2dot     = [b1c_2dot, b2c_2dot, b3c_2dot];
    Omegac      = vee(Rc' * Rc_1dot);
    Omegac_1dot = vee(Rc' * Rc_2dot - hat(Omegac) * hat(Omegac));

    % Attitude errors (Lee 2010 Eq. 10-11)
    eR     = 0.5 * vee(Rc'*state.R - state.R'*Rc);
    eOmega = state.Omega - state.R'*Rc*Omegac;

    % Moment control (Lee 2010 Eq. 13)
    M = -P.kR*eR - P.kOmega*eOmega + cross(state.Omega, J*state.Omega) ...
      - J*(hat(state.Omega)*state.R'*Rc*Omegac - state.R'*Rc*Omegac_1dot);

    % SO(3) error function
    Psi = 0.5 * trace(eye(3) - Rc'*state.R);
end
