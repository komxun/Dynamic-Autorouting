%% IFDS Random-Pair Path Showcase  (GeoJSON building area)
%
%  Generates N random start/finish location pairs inside the GeoJSON
%  building area and computes ONE global IFDS path per pair.
%
%  This script showcases the raw IFDS velocity-field planner only -- there
%  is NO SE(3) trajectory tracker and NO weather constraint (k = 0).
%
%  For each pair the building set is trimmed to the start->finish corridor
%  (cfg.geojson_corridor) so only the relevant buildings are computed by
%  IFDS, exactly as in the main scenario.
%
%  Run from the repository root.
clc, clear, close all
addpath('src', 'plots', 'data')

%% 1. Configuration
cfg = ifds_building_config();              % base parameters (scene 100)
cfg.scene    = 100;
cfg.showDisp = false;                % silence per-path IFDS prints

nPairs    = 10;                      % number of random start/finish pairs
altMin    = 2;                       % [m] AGL minimum start/finish altitude
altMax    = 15;                      % [m] AGL maximum start/finish altitude
corridor  = cfg.geojson_corridor;    % [m] corridor half-width for trimming
clearance = 10;                       % [m] min XY clearance of points from buildings
seed      = 1;                       % RNG seed (set [] for non-reproducible)

if ~isempty(seed), rng(seed); end

%% 2. Load the full building set (untrimmed) and domain extents
[Bfull, dom] = load_buildings_geojson(cfg.geojson_file, ...
    struct('p', cfg.geojson_p, 'pad', cfg.geojson_pad, ...
           'mergeGap', cfg.geojson_mergeGap));
fprintf('Loaded %d buildings | domain [%.0f %.0f] x [%.0f %.0f] m\n', ...
    numel(Bfull), dom.xmin, dom.xmax, dom.ymin, dom.ymax);

% Minimum start<->finish separation so paths are meaningful
minSep = 0.5 * hypot(dom.xmax - dom.xmin, dom.ymax - dom.ymin);

%% 3. Generate random start/finish pairs (clear of buildings)
pairs = zeros(nPairs, 6);            % [x0 y0 z0 x1 y1 z1]
for i = 1:nPairs
    p0 = random_clear_point(Bfull, dom, altMax, clearance);
    while true
        p1 = random_clear_point(Bfull, dom, altMax, clearance);
        if norm(p1 - p0) >= minSep, break; end
    end
    z0 = altMin + rand * (altMax - altMin);   % random start altitude [m AGL]
    z1 = altMin + rand * (altMax - altMin);   % random finish altitude [m AGL]
    pairs(i, :) = [p0, z0, p1, z1];
end

%% 4. Compute one global IFDS path per pair
Paths10  = cell(nPairs, 1);
lengths  = zeros(nPairs, 1);
found    = false(nPairs, 1);
nUsed    = zeros(nPairs, 1);

for i = 1:nPairs
    p0 = pairs(i, 1:2);  z0 = pairs(i, 3);
    p1 = pairs(i, 4:5);  z1 = pairs(i, 6);

    % Trim buildings to this pair's corridor -> only these run in IFDS
    [Bt, ~] = trim_buildings_corridor(Bfull, p0, p1, corridor);
    nUsed(i) = numel(Bt);

    % Build a per-pair config and the matching simulation structures
    cfgi = cfg;
    cfgi.buildings = Bt;
    cfgi.rtsim     = 1;                       % single global path
    cfgi.x_i  = p0(1); cfgi.y_i  = p0(2); cfgi.z_i  = z0;
    cfgi.Xini = p0(1); cfgi.Yini = p0(2); cfgi.Zini = z0;
    cfgi.Xfinal = p1(1); cfgi.Yfinal = p1(2); cfgi.Zfinal = z1;

    [Param, Object, ~, ~, ~] = setup_simulation(cfgi);

    % Single IFDS call (rt = 1) -> one global path, no SE(3) tracking
    Wp = zeros(3, cfgi.tsim + 1);
    Wp(:, 1) = [p0(1); p0(2); z0];
    Paths = cell(1, 1);
    loc_final = [p1(1); p1(2); z1];

    [Paths, Object, len, fp] = IFDS(cfg.rho0, cfg.sigma0, 0, loc_final, 1, ...
        Wp, Paths, Param, 1, Object);

    Paths10{i} = Paths{1, 1};
    lengths(i) = len;
    found(i)   = (fp == 1);

    fprintf('Pair %2d: %2d buildings | path %s | length %.1f m\n', ...
        i, nUsed(i), ternary(found(i), 'FOUND', 'NOT found'), len);
end

%% 5. Plot all buildings + the 10 IFDS paths
colors = lines(nPairs);

figure('Name', 'IFDS Random-Pair Paths', 'Color', 'w');
set(gcf, 'Position', get(0, 'Screensize'));

% ---- (a) 3-D view ----
subplot(1, 3, 1:2); hold on
for j = 1:numel(Bfull)
    draw_building_prism(Bfull(j).foot, 2*Bfull(j).c, [0.8 0.8 0.85], 0.5);
end
plot_paths(Paths10, pairs, colors, found);
axis equal, grid on, view(35, 40)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('IFDS paths (3-D) — %d random pairs', nPairs));
zlim([0 max(2*[Bfull.c]) + 5]);

% ---- (b) Top-down view ----
subplot(1, 3, 3); hold on
for j = 1:numel(Bfull)
    fill(Bfull(j).foot(:, 1), Bfull(j).foot(:, 2), [0.8 0.8 0.85], ...
        'EdgeColor', [0.4 0.4 0.4], 'FaceAlpha', 0.7);
end
plot_paths(Paths10, pairs, colors, found);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('IFDS paths (top-down)');

sgtitle(sprintf('IFDS global paths | corridor = %.0f m', ...
    corridor), 'FontSize', 16);

%% 5b. Validation plot: real GeoJSON footprints (no parallelepiped fitting)
% Overlays the computed paths on the ACTUAL building outlines so the
% viability of the IFDS paths can be checked against the true geometry.
figure('Name', 'IFDS Paths vs. Real GeoJSON Layout', 'Color', 'w');
set(gcf, 'Position', get(0, 'Screensize'));

% ---- (a) 3-D view (real footprints extruded to height) ----
subplot(1, 3, 1:2); hold on
for j = 1:numel(Bfull)
    draw_building_prism(Bfull(j).poly, Bfull(j).height, [0.75 0.78 0.82], 0.6);
end
plot_paths(Paths10, pairs, colors, found);
axis equal, grid on, view(35, 40)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Real GeoJSON layout (3-D)');
zlim([0 max([Bfull.height]) + 5]);

% ---- (b) Top-down view ----
subplot(1, 3, 3); hold on
for j = 1:numel(Bfull)
    fill(Bfull(j).poly(:, 1), Bfull(j).poly(:, 2), [0.75 0.78 0.82], ...
        'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.75, 'FaceAlpha', 0.85);
end
plot_paths(Paths10, pairs, colors, found);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('Real GeoJSON layout (top-down)');

sgtitle(sprintf(['Path viability check — real GeoJSON footprints (no fitting)\n' ...
    'corridor = %.0f m'], corridor), 'FontSize', 15);

%% 5c. Fitting check: parallelepipeds overlaid on the real polygons
% Real footprint (blue, solid) vs. fitted oriented box (red, transparent)
% so the quality of the box fitting can be judged at a glance.
realCol = [0.10 0.35 0.85];      % real polygon colour
boxCol  = [0.90 0.25 0.20];      % fitted parallelepiped colour

figure('Name', 'Parallelepiped Fit vs. Real Polygons', 'Color', 'w');
set(gcf, 'Position', get(0, 'Screensize'));

% ---- (a) 3-D overlay ----
subplot(1, 2, 1); hold on
for j = 1:numel(Bfull)
    draw_building_prism(Bfull(j).poly, Bfull(j).height, realCol, 0.85);   % real
    draw_building_prism(Bfull(j).foot, 2*Bfull(j).c,    boxCol,  0.25);   % fitted box
end
axis equal, grid on, view(35, 40)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Fit overlay (3-D)');
zlim([0 max([Bfull.height]) + 5]);

% ---- (b) Top-down overlay ----
subplot(1, 2, 2); hold on
hReal = []; hBox = [];
for j = 1:numel(Bfull)
    hReal = fill(Bfull(j).poly(:, 1), Bfull(j).poly(:, 2), realCol, ...
        'EdgeColor', realCol*0.6, 'FaceAlpha', 0.85);
    hBox = fill(Bfull(j).foot(:, 1), Bfull(j).foot(:, 2), boxCol, ...
        'EdgeColor', boxCol*0.7, 'LineWidth', 1, 'FaceAlpha', 0.25);
end
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('Fit overlay (top-down)');
legend([hReal, hBox], {'Real polygon', 'Fitted parallelepiped'}, ...
    'Location', 'southoutside');

sgtitle('Parallelepiped fitting vs. real GeoJSON footprints', 'FontSize', 15);

%%
figure()
set(gcf, 'Position', get(0, 'Screensize'));
hold on
for j = 1:numel(Bfull)
    fill(Bfull(j).foot(:, 1), Bfull(j).foot(:, 2), [0.8 0.8 0.85], ...
        'EdgeColor', [0.4 0.4 0.4], 'FaceAlpha', 0.7);
end
plot_paths(Paths10, pairs, colors, found);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');


%% 6. Summary
fprintf('\n===== Summary =====\n');
fprintf('Paths found : %d / %d\n', nnz(found), nPairs);
fprintf('Mean length : %.1f m (found paths)\n', mean(lengths(found)));
disp('====== Completed ======')


%% ======================= Local functions ================================
function p = random_clear_point(B, dom, z, clearance)
% RANDOM_CLEAR_POINT  Uniform random [x y] in the domain, clear of buildings.
    for attempt = 1:1000
        x = dom.xmin + rand * (dom.xmax - dom.xmin);
        y = dom.ymin + rand * (dom.ymax - dom.ymin);
        ok = true;
        for j = 1:numel(B)
            d = hypot(x - B(j).x0, y - B(j).y0) - hypot(B(j).a, B(j).b);
            if d < clearance
                ok = false; break
            end
        end
        if ok, p = [x, y]; return; end
    end
    % Fallback: domain corner if no clear point found
    p = [dom.xmin, dom.ymin];
end

function plot_paths(Paths10, pairs, colors, found)
% PLOT_PATHS  Overlay the IFDS paths with start (o) / finish (x) markers.
    for i = 1:numel(Paths10)
        P = Paths10{i};
        col = colors(i, :);
        if found(i) && ~isempty(P) && size(P, 2) > 1
            plot3(P(1, :), P(2, :), P(3, :), '-', 'Color', col, 'LineWidth', 2);
        end
        % Start / finish markers (random AGL altitudes)
        plot3(pairs(i, 1), pairs(i, 2), pairs(i, 3), 'o', ...
            'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
        plot3(pairs(i, 4), pairs(i, 5), pairs(i, 6), 'x', ...
            'Color', col, 'LineWidth', 2, 'MarkerSize', 10);
    end
end

function zv = path_z(P)
    if isempty(P), zv = 0; else, zv = P(3, 1); end
end

function draw_building_prism(foot, h, fc, fa)
% DRAW_BUILDING_PRISM  Draw an extruded footprint (0..h) as a 3-D prism.
    v = foot;
    if size(v, 1) > 1 && isequal(v(1, :), v(end, :))
        v = v(1:end-1, :);           % drop duplicate closing vertex if present
    end
    n = size(v, 1);
    top = [v, h * ones(n, 1)];
    patch('Vertices', top, 'Faces', 1:n, 'FaceColor', fc, ...
        'FaceAlpha', fa, 'EdgeColor', 'k', 'LineWidth', 0.4);
    bottom = [v, zeros(n, 1)];
    for i = 1:n
        j = mod(i, n) + 1;
        wall = [bottom(i, :); bottom(j, :); top(j, :); top(i, :)];
        patch('Vertices', wall, 'Faces', [1 2 3 4], 'FaceColor', fc, ...
            'FaceAlpha', fa, 'EdgeColor', 'k', 'LineWidth', 0.3);
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

%% Original buildings.geojson file
figure();
set(gcf, 'Position', get(0, 'Screensize'));

for j = 1:numel(Bfull)
    hold on
    fill(Bfull(j).poly(:, 1), Bfull(j).poly(:, 2), [0.75 0.78 0.82], ...
        'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.75, 'FaceAlpha', 0.85);
end
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('Real GeoJSON layout (top-down)');