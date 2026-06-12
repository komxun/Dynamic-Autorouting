%% IFDS Random-Pair Trajectory Showcase  (GeoJSON building area)
%
%  Generates N random start/finish location pairs inside the GeoJSON
%  building area, computes ONE global IFDS path per pair, and then adds a
%  PATH-FOLLOWING layer that flies the IFDS waypoints with the SE(3)
%  geometric trajectory tracker (src/SE3Track.m).
%
%  The resulting SE(3) trajectories are overlaid on the IFDS reference paths
%  in the same plots so the tracking behaviour can be inspected directly.
%
%  This is an extension of ifds_random_paths.m -- there is still NO weather
%  constraint (k = 0); the only addition is the SE(3) trajectory layer.
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
altMax    = 10;                      % [m] AGL maximum start/finish altitude
corridor  = cfg.geojson_corridor;    % [m] corridor half-width for trimming
clearance = 10;                       % [m] min XY clearance of points from buildings
seed      = 1;                       % RNG seed (set [] for non-reproducible)

if ~isempty(seed), rng(seed); end

%% 1b. SE(3) trajectory-tracker parameters
V_ref = cfg.C;                       % [m/s] reference cruise speed along path
Pdyn  = cfg.P;                       % SE(3) controller / quadrotor parameters

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

    [Param, Object, ~, ~, ~, WMCell, dwdxCell, dwdyCell] = setup_simulation(cfgi);

    % Single IFDS call (rt = 1) -> one global path, no SE(3) tracking
    Wp = zeros(3, cfgi.tsim + 1);
    Wp(:, 1) = [p0(1); p0(2); z0];
    Paths = cell(1, 1);
    loc_final = [p1(1); p1(2); z1];

    [Paths, Object, len, fp] = IFDS(cfg.rho0, cfg.sigma0, 0, loc_final, 1, ...
        Wp, Paths, Param, 1, Object, WMCell{15}, dwdxCell{15}, dwdyCell{15});

    Paths10{i} = Paths{1, 1};
    lengths(i) = len;
    found(i)   = (fp == 1);

    fprintf('Pair %2d: %2d buildings | path %s | length %.1f m\n', ...
        i, nUsed(i), ternary(found(i), 'FOUND', 'NOT found'), len);
end

%% 4b. Path-following layer: track each IFDS path with the SE(3) tracker
Traj10  = cell(nPairs, 1);
trajLen = zeros(nPairs, 1);

for i = 1:nPairs
    if ~found(i) || isempty(Paths10{i}) || size(Paths10{i}, 2) < 2
        continue
    end
    Traj = follow_path_se3(Paths10{i}, V_ref, Pdyn);
    Traj10{i} = Traj;
    d = diff(Traj(1:3, :), 1, 2);
    trajLen(i) = sum(sqrt(sum(d.^2, 1)));
    fprintf('Pair %2d: SE(3) trajectory length %.1f m (%d samples)\n', ...
        i, trajLen(i), size(Traj, 2));
end

%% 5. Plot all buildings + the IFDS paths + the SE(3) trajectories
colors = lines(nPairs);

figure('Name', 'IFDS Paths & SE(3) Trajectories', 'Color', 'w');
set(gcf, 'Position', get(0, 'Screensize'));

% ---- (a) 3-D view ----
subplot(1, 2, 1); hold on
for j = 1:numel(Bfull)
    draw_building_prism(Bfull(j).foot, 2*Bfull(j).c, [0.8 0.8 0.85], 0.5);
end
plot_paths(Paths10, pairs, colors, found);
plot_trajectories(Traj10, colors);
axis equal, grid on, view(35, 40)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('IFDS paths (--) & SE(3) trajectories (-) — %d random pairs', nPairs));
zlim([0 max(2*[Bfull.c]) + 5]);

% ---- (b) Top-down view ----
subplot(1, 2, 2); hold on
for j = 1:numel(Bfull)
    fill(Bfull(j).foot(:, 1), Bfull(j).foot(:, 2), [0.8 0.8 0.85], ...
        'EdgeColor', [0.4 0.4 0.4], 'FaceAlpha', 0.7);
end
plot_paths(Paths10, pairs, colors, found);
plot_trajectories(Traj10, colors);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('IFDS paths & SE(3) trajectories (top-down)');

sgtitle(sprintf(['IFDS global paths (dashed) vs. SE(3) trajectories (solid)\n' ...
    'corridor = %.0f m | V_{ref} = %.1f m/s'], ...
    corridor, V_ref), 'FontSize', 15);
%%
figure()
set(gcf, 'Position', get(0, 'Screensize'));
hold on
for j = 1:numel(Bfull)
    fill(Bfull(j).foot(:, 1), Bfull(j).foot(:, 2), [0.8 0.8 0.85], ...
        'EdgeColor', [0.4 0.4 0.4], 'FaceAlpha', 0.7);
end
plot_paths(Paths10, pairs, colors, found);
plot_trajectories(Traj10, colors);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');

title(sprintf(['IFDS global paths (dashed) vs. SE(3) trajectories (solid)\n' ...
    'corridor = %.0f m | V_{ref} = %.1f m/s'], ...
    corridor, V_ref), 'FontSize', 15);

%% 5b. Validation plot: real GeoJSON footprints (no parallelepiped fitting)
% Overlays the computed paths/trajectories on the ACTUAL building outlines.
figure('Name', 'SE(3) Trajectories vs. Real GeoJSON Layout', 'Color', 'w');
set(gcf, 'Position', get(0, 'Screensize'));

% ---- (a) 3-D view (real footprints extruded to height) ----
subplot(1, 3, 1:2); hold on
for j = 1:numel(Bfull)
    draw_building_prism(Bfull(j).poly, Bfull(j).height, [0.75 0.78 0.82], 0.6);
end
plot_paths(Paths10, pairs, colors, found);
plot_trajectories(Traj10, colors);
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
plot_trajectories(Traj10, colors);
axis equal, grid on, view(0, 90)
xlabel('X [m]'); ylabel('Y [m]');
title('Real GeoJSON layout (top-down)');

sgtitle(sprintf(['SE(3) trajectory viability — real GeoJSON footprints (no fitting)\n' ...
    'corridor = %.0f m'], corridor), 'FontSize', 15);

%% 6. Export SE(3) trajectories as GCS waypoint files (QGC WPL 110)
% Converts each flown trajectory from the local metric frame back to WGS-84
% lon/lat and writes one ArduPilot-compatible mission file per pair. The
% format is read by Mission Planner and QGroundControl and flyable on
% ArduPilot (cruise altitude is relative-to-home / AGL).
wpSpacing = 10;        % [m] min horizontal spacing between exported waypoints
wpDir     = fullfile('output', 'waypoints');
if ~exist(wpDir, 'dir'), mkdir(wpDir); end

nExported = 0;
for i = 1:nPairs
    T = Traj10{i};
    if isempty(T) || size(T, 2) < 2, continue; end

    % Downsample by arc length to keep the mission size manageable
    keep = downsample_by_distance(T, wpSpacing);
    Tk   = T(:, keep);

    % Local metric (world) frame -> WGS-84 geocoordinates
    [lat, lon] = local_to_geo(Tk(1, :), Tk(2, :), dom.geo);
    alt = Tk(3, :);                 % [m] altitude relative to home (AGL)

    % SE(3) desired heading psi_d [rad, math: CCW from +X=East] ->
    % compass yaw [deg, CW from North] for the MAVLink waypoint (param4)
    yaw = mod(90 - rad2deg(Tk(4, :)), 360);

    fname = fullfile(wpDir, sprintf('pair_%02d.waypoints', i));
    write_qgc_wpl(fname, lat, lon, alt, yaw);
    nExported = nExported + 1;
    fprintf('Exported pair %2d -> %s (%d waypoints)\n', i, fname, numel(lat));
end
fprintf('Wrote %d waypoint file(s) to %s\n', nExported, wpDir);

%% 7. Summary
fprintf('\n===== Summary =====\n');
fprintf('Paths found     : %d / %d\n', nnz(found), nPairs);
fprintf('Mean IFDS length: %.1f m (found paths)\n', mean(lengths(found)));
fprintf('Mean SE(3) length: %.1f m (tracked paths)\n', mean(trajLen(trajLen > 0)));
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

function keep = downsample_by_distance(T, spacing)
% DOWNSAMPLE_BY_DISTANCE  Indices into T (3xN) that keep the first and last
% point plus intermediate points at least 'spacing' metres apart (XY).
    N = size(T, 2);
    keep = 1;
    last = T(1:2, 1);
    for j = 2:N-1
        if norm(T(1:2, j) - last) >= spacing
            keep(end+1) = j; %#ok<AGROW>
            last = T(1:2, j);
        end
    end
    keep(end+1) = N;                  % always keep the final point
    keep = unique(keep, 'stable');
end

function [lat, lon] = local_to_geo(X, Y, geo)
% LOCAL_TO_GEO  Invert the equirectangular projection used by
% load_buildings_geojson: world metric XY [m] -> WGS-84 lat/lon [deg].
    lon = geo.lon0 + (X - geo.offset(1)) / geo.mPerLon;
    lat = geo.lat0 + (Y - geo.offset(2)) / geo.mPerLat;
end

function write_qgc_wpl(fname, lat, lon, alt, yaw)
% WRITE_QGC_WPL  Write a QGroundControl WPL 110 mission file (also read by
% Mission Planner and flyable by ArduPilot). Tab-separated columns are:
%   idx  current  frame  command  p1 p2 p3 p4  lat  lon  alt  autocontinue
% The SE(3) desired heading is written into param4 (yaw, deg, compass).
    if nargin < 5 || isempty(yaw), yaw = nan(size(lat)); end
    fid = fopen(fname, 'w');
    if fid < 0, error('Cannot open %s for writing', fname); end
    closer = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, 'QGC WPL 110\n');

    % MAVLink frames / commands
    FRAME_GLOBAL  = 0;     % absolute alt   (used for HOME row)
    FRAME_REL_ALT = 3;     % alt relative to home (AGL)
    CMD_WAYPOINT  = 16;    % MAV_CMD_NAV_WAYPOINT
    CMD_TAKEOFF   = 22;    % MAV_CMD_NAV_TAKEOFF
    CMD_LAND      = 21;    % MAV_CMD_NAV_LAND

    idx = 0;
    % Row 0: HOME at the first point (Mission Planner uses line 0 as home)
    fprintf(fid, '%d\t1\t%d\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t%.6f\t1\n', ...
        idx, FRAME_GLOBAL, CMD_WAYPOINT, lat(1), lon(1), alt(1));
    idx = idx + 1;

    % Row 1: TAKEOFF to the first waypoint altitude (yaw -> param4)
    fprintf(fid, '%d\t0\t%d\t%d\t0\t0\t0\t%.6f\t%.8f\t%.8f\t%.6f\t1\n', ...
        idx, FRAME_REL_ALT, CMD_TAKEOFF, yaw(1), lat(1), lon(1), alt(1));
    idx = idx + 1;

    % Cruise waypoints (relative altitude, desired heading -> param4)
    for k = 1:numel(lat)
        fprintf(fid, '%d\t0\t%d\t%d\t0\t0\t0\t%.6f\t%.8f\t%.8f\t%.6f\t1\n', ...
            idx, FRAME_REL_ALT, CMD_WAYPOINT, yaw(k), lat(k), lon(k), alt(k));
        idx = idx + 1;
    end

    % Final: LAND at the last point
    fprintf(fid, '%d\t0\t%d\t%d\t0\t0\t0\t%.6f\t%.8f\t%.8f\t0.000000\t1\n', ...
        idx, FRAME_REL_ALT, CMD_LAND, yaw(end), lat(end), lon(end));
end

function Traj = follow_path_se3(Pwp, V, Pdyn)
% FOLLOW_PATH_SE3  Track an IFDS path (3xN waypoints) with the SE(3)
% geometric tracker (SE3Track), returning the full flown trajectory as a
% 4xM array: rows 1-3 are inertial position [m], row 4 is the SE(3) desired
% heading psi_d [rad] (atan2 of the active segment direction, X=E, Y=N).
    if isequal(Pwp(:, 1), Pwp(:, end)) && size(Pwp, 2) > 1
        Pwp = Pwp(:, 1:end-1);
    end
    N = size(Pwp, 2);

    % Fresh SE(3) state at the path start, cruising along the first segment
    d0 = Pwp(:, 2) - Pwp(:, 1);
    if norm(d0) > 1e-9, d0 = d0 / norm(d0); else, d0 = [1; 0; 0]; end
    state.p     = Pwp(:, 1);
    state.v     = V * d0;
    state.R     = eye(3);
    state.Omega = zeros(3, 1);

    % Dirty-derivative filters (as in setup_simulation)
    filters.dv1dt = DirtyDerivative(1, Pdyn.tau,    Pdyn.Ts);
    filters.dv2dt = DirtyDerivative(2, Pdyn.tau*10, Pdyn.Ts);

    % Empty SE(3) telemetry logger (mutated in place by SE3Track)
    logger = struct('t', [], 'x', [], 'xd', [], 'v', [], 'vd', [], ...
        'Omega', [], 'Omegac', [], 'Psi', [], 'f', [], 'M', [], 'deltaF', []);

    psi0 = atan2(d0(2), d0(1));            % desired heading at the start
    Traj = [state.p; psi0];
    for j = 1:N-1
        Wi = Pwp(:, j);  Wf = Pwp(:, j+1);
        segLen = norm(Wf - Wi);
        if segLen < 1e-6, continue; end

        % Skip waypoints already behind the UAV
        if dot(Wf - Wi, state.p - Wf) >= 0, continue; end

        % SE(3) desired heading for this segment (matches SE3Track's psi_d)
        d_hat = (Wf - Wi) / segLen;
        psi_d = atan2(d_hat(2), d_hat(1));

        % Generous per-segment time budget so the tracker reaches the plane
        dt_max = segLen / max(V, eps) * 4 + 2;
        [pos_seg, ~, state, filters, ~, logger] = ...
            SE3Track(Wi, Wf, state, filters, V, dt_max, Pdyn, logger);
        if size(pos_seg, 2) > 1
            n_new = size(pos_seg, 2) - 1;
            Traj = [Traj, [pos_seg(:, 2:end); repmat(psi_d, 1, n_new)]]; %#ok<AGROW>
        end
    end
end

function plot_paths(Paths10, pairs, colors, found)
% PLOT_PATHS  Overlay the IFDS paths with start (o) / finish (x) markers.
    for i = 1:numel(Paths10)
        P = Paths10{i};
        col = colors(i, :);
        if found(i) && ~isempty(P) && size(P, 2) > 1
            plot3(P(1, :), P(2, :), P(3, :), '--', 'Color', col, 'LineWidth', 1);
        end
        % Start / finish markers (random AGL altitudes)
        plot3(pairs(i, 1), pairs(i, 2), pairs(i, 3), 'o', ...
            'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
        plot3(pairs(i, 4), pairs(i, 5), pairs(i, 6), 'x', ...
            'Color', col, 'LineWidth', 2, 'MarkerSize', 10);
    end
end

function plot_trajectories(Traj10, colors)
% PLOT_TRAJECTORIES  Overlay the SE(3)-flown trajectories (solid lines).
    for i = 1:numel(Traj10)
        T = Traj10{i};
        if isempty(T) || size(T, 2) < 2, continue; end
        plot3(T(1, :), T(2, :), T(3, :), '-', ...
            'Color', colors(i, :), 'LineWidth', 2);
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
