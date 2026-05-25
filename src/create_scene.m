function Obj = create_scene(num, Obj, X, Y, Z, rt, alpha_deg)
% CREATE_SCENE  Define obstacle configurations for each scenario number.
%
%   Obj = create_scene(sceneNum, Obj, X, Y, Z, rt, alpha_deg)
%
%   Each case calls create_shape with (X, Y, Z, x0, y0, z0, a, b, c, p, q, r, rt, alpha_deg, Obj)
%   Shape types are distinguished by the (p, q, r) exponents:
%     Sphere   : p=1, q=1, r=1
%     Cylinder : p=1, q=1, r=4
%     Cone     : p=1, q=1, r=0.5
%     Pipe     : p=2, q=2, r=2
%     Ceiling  : p=20, q=20, r=20

    switch num
        case 0
            % Ceiling
            z0c = 50 + 10;   % z0 = 50, h = 10 → offset = z0 + h
            Obj(1) = create_shape(X, Y, Z, 100, 0, z0c, 100, 100, 10, 20, 20, 20, rt, alpha_deg, Obj(1));

        case 1  % Single sphere
            Obj(1) = create_shape(X, Y, Z, 100, 0, 50, 25, 25, 25, 1, 1, 1, rt, alpha_deg, Obj(1));

        case 2  % 2 objects
            Obj(1) = create_shape(X, Y, Z, 60, 5, 0, 15, 15, 50, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 120, -10, 0, 25, 25, 25, 1, 1, 1, rt, alpha_deg, Obj(2));

        case 3  % 3 objects
            Obj(1) = create_shape(X, Y, Z, 60, 5, 0, 15, 15, 50, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 120, -10, 0, 25, 25, 25, 1, 1, 1, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 168, 0, 0, 12.5, 12.5, 80, 1, 1, 0.5, rt, alpha_deg, Obj(3));

        case 4  % 3 complex objects
            Obj(1) = create_shape(X, Y, Z, 100, 5, 0, 12.5, 12.5, 200, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 60, 20, 60, 40, 40, 5, 2, 2, 2, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 130, -30, 30, 50, 50, 50, 2, 2, 2, rt, alpha_deg, Obj(3));

        case 5
            Obj(1) = create_shape(X, Y, Z, 50, -20, 0, 15, 15, 50, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 100, -20, 0, 15, 15, 50, 1, 1, 0.5, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 150, -20, 0, 15, 15, 50, 2, 2, 2, rt, alpha_deg, Obj(3));

        case 7  % 7 objects
            Obj(1) = create_shape(X, Y, Z, 60, 8, 0, 35, 35, 50, 1, 1, 0.5, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 100, -24, 0, 44.5, 44.5, 100, 1, 1, 0.5, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 160, 40, -4, 50, 50, 30, 1, 1, 0.5, rt, alpha_deg, Obj(3));
            Obj(4) = create_shape(X, Y, Z, 100, 100, -10, 75, 75, 100, 1, 1, 0.5, rt, alpha_deg, Obj(4));
            Obj(5) = create_shape(X, Y, Z, 180, -70, -10, 75, 75, 20, 1, 1, 0.5, rt, alpha_deg, Obj(5));
            Obj(6) = create_shape(X, Y, Z, 75, -75, -10, 75, 75, 40, 1, 1, 0.5, rt, alpha_deg, Obj(6));
            Obj(7) = create_shape(X, Y, Z, 170, -6, 0, 17, 17, 100, 1, 1, 4, rt, alpha_deg, Obj(7));

        case 12  % 12 objects (urban)
            Obj(1)  = create_shape(X, Y, Z, 100, 5, 0, 15, 15, 50, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2)  = create_shape(X, Y, Z, 140, 20, 0, 20, 20, 10, 2, 2, 2, rt, alpha_deg, Obj(2));
            Obj(3)  = create_shape(X, Y, Z, 20, 20, 0, 12, 12, 40, 2, 2, 2, rt, alpha_deg, Obj(3));
            Obj(4)  = create_shape(X, Y, Z, 55, -20, 0, 14, 14, 50, 2, 2, 2, rt, alpha_deg, Obj(4));
            Obj(5)  = create_shape(X, Y, Z, 53, -60, 0, 25, 25, 25, 1, 1, 1, rt, alpha_deg, Obj(5));
            Obj(6)  = create_shape(X, Y, Z, 150, -80, 0, 20, 20, 50, 2, 2, 2, rt, alpha_deg, Obj(6));
            Obj(7)  = create_shape(X, Y, Z, 100, -35, 0, 25, 25, 45, 1, 1, 0.5, rt, alpha_deg, Obj(7));
            Obj(8)  = create_shape(X, Y, Z, 170, 2, 0, 10, 10, 50, 1, 1, 0.5, rt, alpha_deg, Obj(8));
            Obj(9)  = create_shape(X, Y, Z, 60, 35, 0, 25, 25, 30, 1, 1, 0.5, rt, alpha_deg, Obj(9));
            Obj(10) = create_shape(X, Y, Z, 110, 70, 0, 30, 30, 50, 1, 1, 4, rt, alpha_deg, Obj(10));
            Obj(11) = create_shape(X, Y, Z, 170, 60, 0, 20, 20, 27, 2, 2, 2, rt, alpha_deg, Obj(11));
            Obj(12) = create_shape(X, Y, Z, 150, -30, 0, 16, 16, 45, 1, 1, 0.5, rt, alpha_deg, Obj(12));

        case 41  % Dynamic: 3 objects orbiting
            Obj(1) = create_shape(X, Y, Z, 100+50*sin(rt/8), 0+50*cos(rt/8), 0, 10, 10, 80, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 100, 0, 0, 15, 15, 15, 1, 1, 1, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 100-50*sin(rt/8), 0-50*cos(rt/8), 0, 10, 10, 50, 1, 1, 4, rt, alpha_deg, Obj(3));

        case 42  % Dynamic: 4 objects
            Oy1 = -5 + 60*cos(0.4*single(rt));
            Oy2 = -20 - 20*sin(0.8*single(rt));
            Oz2 =  60 + 20*cos(0.8*single(rt));
            Obj(1) = create_shape(X, Y, Z, 40, 5, 0, 15, 15, 40, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 120, -10, 0, 12.5, 12.5, 80, 1, 1, 0.5, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 80, Oy1, 0, 5, 5, 60, 1, 1, 4, rt, alpha_deg, Obj(3));
            Obj(4) = create_shape(X, Y, Z, 160, Oy2, Oz2, 10, 10, 10, 1, 1, 1, rt, alpha_deg, Obj(4));

        case 44  % Dynamic: 7 objects
            Oy1 = 0 + 60*sin(0.7*single(rt));
            Oy2 = 0 + 60*cos(0.7*single(rt));
            shift = 40*sin(0.5*single(rt));
            Obj(1) = create_shape(X, Y, Z, 40, 5, 0, 15, 15, 80, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 40, -50, 0, 20, 20, 30, 2, 2, 2, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 150, 50, 0, 20, 20, 60, 2, 2, 2, rt, alpha_deg, Obj(3));
            Obj(4) = create_shape(X, Y, Z, 150, -10, 0, 15, 15, 80, 1, 1, 4, rt, alpha_deg, Obj(4));
            Obj(5) = create_shape(X, Y, Z, 110, Oy1, 0, 10, 10, 50, 1, 1, 4, rt, alpha_deg, Obj(5));
            Obj(6) = create_shape(X, Y, Z, 80, Oy2, 0, 15, 15, 30, 2, 2, 2, rt, alpha_deg, Obj(6));
            Obj(7) = create_shape(X, Y, Z, 100+shift, 0+shift, 60, 15, 15, 15, 1, 1, 1, rt, alpha_deg, Obj(7));

        case 69
            Obj(1) = create_shape(X, Y, Z, 100, 5, 0, 15, 15, 80, 1, 1, 4, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 100, 30, 0, 20, 20, 20, 1, 1, 1, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 100, -20, 0, 20, 20, 20, 1, 1, 1, rt, alpha_deg, Obj(3));
            Obj(4) = create_shape(X, Y, Z, 100, 5, 80, 15, 15, 15, 1, 1, 1, rt, alpha_deg, Obj(4));

        case 6969  % Dynamic: 3 spheres orbiting
            Obj(1) = create_shape(X, Y, Z, 100+30*sin(rt/8), 0+30*cos(rt/8), 0, 20, 20, 20, 1, 1, 1, rt, alpha_deg, Obj(1));
            Obj(2) = create_shape(X, Y, Z, 100, 0, 0, 20, 20, 80, 1, 1, 4, rt, alpha_deg, Obj(2));
            Obj(3) = create_shape(X, Y, Z, 100-30*sin(rt/8), 0-30*cos(rt/8), 0, 20, 20, 20, 1, 1, 1, rt, alpha_deg, Obj(3));
    end
end
