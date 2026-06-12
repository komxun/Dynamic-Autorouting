function [Gamma, Gamma_star] = PlotObject(Object, Rg, rt, rtsim, X, Y, Z, Gamma, Gamma_star)
    nObj = size(Object, 2);

    % --- Expand axis limits to enclose all obstacles (never shrink) ---
    xs = []; ys = []; zs = [];
    for j = 1:nObj
        o = Object(j).origin(rt, :);
        xs = [xs, o(1) - Object(j).a, o(1) + Object(j).a]; %#ok<AGROW>
        ys = [ys, o(2) - Object(j).b, o(2) + Object(j).b]; %#ok<AGROW>
        zs = [zs, o(3) - Object(j).c, o(3) + Object(j).c]; %#ok<AGROW>
    end
    pad = 10;
    cx = xlim; cy = ylim; cz = zlim;
    xlim([min([cx(1), min(xs) - pad]),        max([cx(2), max(xs) + pad])]);
    ylim([min([cy(1), min(ys) - pad]),        max([cy(2), max(ys) + pad])]);
    zlim([max(0, min([cz(1), min(zs) - pad])), max([cz(2), max(zs) + pad])]);

    for j = 1:nObj
        x0 = Object(j).origin(rt, 1);
        y0 = Object(j).origin(rt, 2);
        z0 = Object(j).origin(rt, 3);
        a = Object(j).a;
        b = Object(j).b;
        c = Object(j).c;
        p = Object(j).p;
        q = Object(j).q;
        r = Object(j).r;

        if isfield(Object, 'yaw') && ~isempty(Object(j).yaw)
            yaw = Object(j).yaw;
        else
            yaw = 0;
        end

        Rstar = Object(j).Rstar;

        % In-plane rotation (world -> local box frame)
        Xl =  cos(yaw)*(X - x0) + sin(yaw)*(Y - y0);
        Yl = -sin(yaw)*(X - x0) + cos(yaw)*(Y - y0);

        Gamma(X, Y, Z) = (Xl / a).^(2*p) + (Yl / b).^(2*q) + ((Z - z0) / c).^(2*r);
        Gamma_star(X, Y, Z) = Gamma - ( (Rstar + Rg)/Rstar )^2 + 1;

        fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1,'MeshDensity',100, 'FaceColor', 'w'), hold on
        fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',100, 'FaceColor', 'w')
    end

end