function Obj = create_shape(X, Y, Z, x0, y0, z0, a, b, c, p, q, r, rt, alpha_deg, Obj, yaw)
% CREATE_SHAPE  Unified IFDS shape primitive.
%
%   Computes the implicit-surface boundary function Gamma and its gradient
%   for a super-ellipsoid centred at (x0, y0, z0) with semi-axes a, b, c
%   and exponent indices p, q, r.
%
%   Gamma = (Xl/a)^(2p) + (Yl/b)^(2q) + ((Z-z0)/c)^(2r)
%
%   where (Xl, Yl) are the in-plane coordinates rotated by -yaw (radians)
%   about the obstacle centre, enabling oriented (rotated) boxes. yaw is
%   optional and defaults to 0 (axis-aligned, backward compatible).
%
%   The tangent vector t is rotated by alpha_deg (in degrees) around the
%   gradient direction to enable the shape-following feature.

    if nargin < 16 || isempty(yaw)
        yaw = 0;
    end

    % In-plane rotation (world -> local box frame): rotate by -yaw
    cy = cos(yaw);  sy = sin(yaw);
    dx = X - x0;    dy = Y - y0;
    Xl =  cy*dx + sy*dy;
    Yl = -sy*dx + cy*dy;

    % Boundary function
    Gamma = (Xl / a).^(2*p) + (Yl / b).^(2*q) + ((Z - z0) / c).^(2*r);

    % Gradient w.r.t. local coords
    gXl = (2*p*(Xl/a).^(2*p - 1)) / a;
    gYl = (2*q*(Yl/b).^(2*q - 1)) / b;

    % Gradient (normal direction) mapped back to world frame
    dGdx = gXl*cy - gYl*sy;
    dGdy = gXl*sy + gYl*cy;
    dGdz = (2*r*((Z - z0)/c).^(2*r - 1)) / c;

    n = [dGdx; dGdy; dGdz];

    % Tangent vector with alpha rotation
    alpha = alpha_deg * pi / 180;
    rot = [dGdy,  dGdx*dGdz, dGdx;
          -dGdx,  dGdy*dGdz, dGdy;
           0,    -(dGdx^2)-(dGdy^2), dGdz];
    tprime = [cos(alpha); sin(alpha); 0];
    t = rot * tprime;

    % Save to Object struct
    Obj.origin(rt,:) = [x0, y0, z0];
    Obj.Gamma  = Gamma;
    Obj.n      = n;
    Obj.t      = t;
    Obj.a      = a;
    Obj.b      = b;
    Obj.c      = c;
    Obj.p      = p;
    Obj.q      = q;
    Obj.r      = r;
    Obj.yaw    = yaw;
    Obj.Rstar  = min([a, b, c]);
end
