function Obj = create_shape(X, Y, Z, x0, y0, z0, a, b, c, p, q, r, rt, alpha_deg, Obj)
% CREATE_SHAPE  Unified IFDS shape primitive.
%
%   Computes the implicit-surface boundary function Gamma and its gradient
%   for a super-ellipsoid centred at (x0, y0, z0) with semi-axes a, b, c
%   and exponent indices p, q, r.
%
%   Gamma = ((X-x0)/a)^(2p) + ((Y-y0)/b)^(2q) + ((Z-z0)/c)^(2r)
%
%   The tangent vector t is rotated by alpha_deg (in degrees) around the
%   gradient direction to enable the shape-following feature.

    % Boundary function
    Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);

    % Gradient (normal direction)
    dGdx = (2*p*((X - x0)/a).^(2*p - 1)) / a;
    dGdy = (2*q*((Y - y0)/b).^(2*q - 1)) / b;
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
    Obj.Rstar  = min([a, b, c]);
end
