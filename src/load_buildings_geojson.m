function [B, dom] = load_buildings_geojson(file, opts)
% LOAD_BUILDINGS_GEOJSON  Parse a GeoJSON building footprint file and fit an
% oriented parallelepiped (box) to each building footprint.
%
%   [B, dom] = load_buildings_geojson(file, opts)
%
%   Each GeoJSON Polygon feature is:
%     1. converted from [lon lat] (WGS-84) to a local east-north metric
%        frame using an equirectangular projection about the mean lat/lon
%        (no Aerospace Toolbox dependency, accurate over a few hundred m),
%     2. fitted with the minimum-area enclosing rectangle (rotating
%        calipers over the convex hull) -> centre, half-extents and yaw,
%     3. extruded to the building 'height' property to form a box that is
%        represented in IFDS as an oriented super-ellipsoid prism.
%
%   The whole scene is translated so all coordinates are positive (a margin
%   is kept from the origin), matching the convention in CA_Lee2026.
%
%   Footprints that overlap or lie within opts.mergeGap metres of each other
%   (e.g. terraced/side-by-side buildings separated by a thin wall or narrow
%   street) are grouped and fitted with a SINGLE oriented box, so adjacent
%   buildings become one large rectangle instead of several abutting ones.
%
%   Inputs:
%     file  - path to the .geojson file
%     opts  - optional struct:
%               .margin    [m] gap kept between scene and axes origin (def 20)
%               .pad       [m] lateral safety added to box half-extents (def 2)
%               .p         super-ellipsoid exponent index (def 4 -> ^8 box)
%               .minHeight [m] floor for missing/zero heights (def 3)
%               .mergeGap  [m] footprints within this gap are merged into one
%                          box (def 1.5; set 0 to merge only touching ones)
%
%   Outputs:
%     B   - struct array (one per building) with fields:
%             x0,y0,z0  box centre [m]
%             a,b,c     box half-extents (local x, local y, z) [m]
%             p,q,r     super-ellipsoid exponent indices
%             yaw       in-plane orientation [rad]
%             foot      5x2 closed footprint corners (world frame) [m]
%             height    building height [m]
%     dom - struct with scene extents and a suggested start/finish pair:
%             xmin,xmax,ymin,ymax,zmax
%             start  [x y z] just outside the field (low-x side)
%             final  [x y z] just outside the field (high-x side)

    if nargin < 2 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'margin'),    opts.margin    = 20;  end
    if ~isfield(opts, 'pad'),       opts.pad       = 2;   end
    if ~isfield(opts, 'p'),         opts.p         = 4;   end
    if ~isfield(opts, 'minHeight'), opts.minHeight = 3;   end
    if ~isfield(opts, 'mergeGap'),  opts.mergeGap  = 1.5; end

    lateral_only = true;

    geo = jsondecode(fileread(file));
    features = geo.features;
    n = numel(features);

    % ---- First pass: gather rings, heights and reference origin ----
    rings   = cell(n, 1);
    heights = zeros(n, 1);
    lons = []; lats = [];
    for ii = 1:n
        if iscell(features), feat = features{ii}; else, feat = features(ii); end
        % ring is a set of building's vertices coordinates
        ring = feat.geometry.coordinates;
        % if data type of ring is cell, convert to double
        while iscell(ring), ring = ring{1}; end
        % if ring has 3 dimensions (usually), squeeze into 2 dimensions
        if ndims(ring) == 3, ring = squeeze(ring(1, :, :)); end
        % Delete the duplicated start/end vertices of the building
        if size(ring, 1) > 1 && isequal(ring(1, :), ring(end, :))
            ring(end, :) = [];
        end
        rings{ii} = ring;
        
        if lateral_only
            heights(ii) = 999;
        else
            if isfield(feat.properties, 'height') && ~isempty(feat.properties.height)
                heights(ii) = feat.properties.height;
            else
                % if building heights are not specified, assume minHeight
                heights(ii) = opts.minHeight;
            end
        end

        lons = [lons; ring(:, 1)]; %#ok<AGROW>
        lats = [lats; ring(:, 2)]; %#ok<AGROW>
    end

    lat0 = mean(lats);  lon0 = mean(lons);
    mPerLat = 111320;
    mPerLon = 111320 * cosd(lat0);

    % ---- Second pass: project every footprint to metres ----
    polysXY = cell(n, 1);
    for ii = 1:n
        ring = rings{ii};
        polysXY{ii} = [(ring(:, 1) - lon0) * mPerLon, ...
                       (ring(:, 2) - lat0) * mPerLat];
    end

    % ---- Merge footprints within opts.mergeGap into single groups ----
    % Side-by-side buildings (touching, or separated by a thin wall/street up
    % to mergeGap metres) are grouped and fitted with ONE oriented box.
    groups = merge_footprint_groups(polysXY, opts.mergeGap);
    m = numel(groups);
    fprintf('Merged %d footprints into %d box group(s) (mergeGap = %.1f m)\n', ...
        n, m, opts.mergeGap);

    % B contains a repeated placeholder struct, one per MERGED group
    B = repmat(struct('x0', 0, 'y0', 0, 'z0', 0, 'a', 0, 'b', 0, 'c', 0, ...
        'p', opts.p, 'q', opts.p, 'r', opts.p, 'yaw', 0, 'foot', [], ...
        'poly', [], 'height', 0), m, 1);

    allXY = [];
    for gi = 1:m
        idx = groups{gi};

        % Combined vertices of every footprint in the group
        XY = vertcat(polysXY{idx});
        x  = XY(:, 1);  y = XY(:, 2);

        [cx, cy, ha, hb, ang, corners] = min_area_rect(x, y);
        h = max([heights(idx); opts.minHeight]);

        B(gi).x0  = cx;          
        B(gi).y0  = cy;          
        B(gi).z0 = 0;  %h/2
        B(gi).a   = ha + opts.pad;
        B(gi).b   = hb + opts.pad;
        B(gi).c   = h; %h/2
        B(gi).p   = opts.p;      
        B(gi).q   = opts.p;      
        B(gi).r  = opts.p;
        B(gi).yaw = ang;
        B(gi).foot   = corners;
        B(gi).poly   = group_outline(polysXY(idx));   % real merged footprint
        B(gi).height = h;

        allXY = [allXY; x, y]; %#ok<AGROW>
    end

    % ---- Translate scene so everything is positive (keep margin) ----
    gmin = min(allXY, [], 1);
    off  = -gmin + opts.margin;
    for ii = 1:numel(B)
        B(ii).x0   = B(ii).x0 + off(1);
        B(ii).y0   = B(ii).y0 + off(2);
        B(ii).foot = B(ii).foot + off;
        B(ii).poly = B(ii).poly + off;
    end

    % ---- Scene extents and auto start/finish (just outside the field) ----
    xs = [B.x0];  ys = [B.y0];
    as = [B.a];   bs = [B.b];
    dom.xmin = min(xs - as);   dom.xmax = max(xs + as);
    dom.ymin = min(ys - bs);   dom.ymax = max(ys + bs);
    dom.zmax = max([B.z0] + [B.c]);

    ymid  = (dom.ymin + dom.ymax) / 2;
    clear2 = opts.margin;                 % stand-off distance from the field
    dom.start = [dom.xmin - clear2, ymid, NaN];   % z filled in by caller
    dom.final = [dom.xmax + clear2, ymid, NaN];

    % ---- Georeference (invert: world XY [m] -> WGS-84 lon/lat) ----
    %   lon = lon0 + (Xworld - offset(1)) / mPerLon
    %   lat = lat0 + (Yworld - offset(2)) / mPerLat
    dom.geo.lat0    = lat0;
    dom.geo.lon0    = lon0;
    dom.geo.mPerLat = mPerLat;
    dom.geo.mPerLon = mPerLon;
    dom.geo.offset  = off;                % [ox oy] added to local frame [m]
end

% =========================================================================
function [cx, cy, ha, hb, ang, corners] = min_area_rect(x, y)
% MIN_AREA_RECT  Minimum-area enclosing rectangle via rotating calipers.
%
%   Returns the rectangle centre (cx, cy), half-extents (ha along local x,
%   hb along local y), orientation ang [rad] of the local x-axis, and the
%   5x2 closed corner list in the world frame.

    x = x(:);  y = y(:);

    % Degenerate footprints -> axis-aligned bounding box
    if numel(x) < 3
        cx = mean(x);  cy = mean(y);
        ha = max((max(x) - min(x)) / 2, 0.5);
        hb = max((max(y) - min(y)) / 2, 0.5);
        ang = 0;
        corners = box_corners(cx, cy, ha, hb, ang);
        return
    end

    try
        k = convhull(x, y);
    catch
        k = (1:numel(x))';  k(end + 1) = 1;
    end
    hx = x(k);  hy = y(k);     % closed convex polygon

    bestArea = inf;
    cx = mean(x); cy = mean(y); ha = 1; hb = 1; ang = 0;
    for i = 1:numel(hx) - 1
        ex = hx(i + 1) - hx(i);
        ey = hy(i + 1) - hy(i);
        if ex == 0 && ey == 0, continue; end

        th = atan2(ey, ex);
        c  = cos(-th);  s = sin(-th);
        xr =  c*hx - s*hy;     % rotate hull into edge-aligned frame
        yr =  s*hx + c*hy;

        w = max(xr) - min(xr);
        h = max(yr) - min(yr);
        A = w * h;
        if A < bestArea
            bestArea = A;
            ha  = w / 2;  hb = h / 2;  ang = th;
            cxr = (max(xr) + min(xr)) / 2;
            cyr = (max(yr) + min(yr)) / 2;
            cb  = cos(th);  sb = sin(th);     % rotate centre back to world
            cx  = cb*cxr - sb*cyr;
            cy  = sb*cxr + cb*cyr;
        end
    end

    corners = box_corners(cx, cy, ha, hb, ang);
end

% =========================================================================
function corners = box_corners(cx, cy, ha, hb, ang)
    R   = [cos(ang), -sin(ang); sin(ang), cos(ang)];
    loc = [ ha,  hb;  -ha,  hb;  -ha, -hb;   ha, -hb;   ha,  hb];
    corners = (R * loc')' + [cx, cy];
end

% =========================================================================
function groups = merge_footprint_groups(polys, gap)
% MERGE_FOOTPRINT_GROUPS  Group footprint polygons that overlap or lie within
% 'gap' metres of one another (connected components of the within-gap
% adjacency graph). Returns a cell array of index vectors into 'polys'.
    n = numel(polys);
    if n == 0, groups = {}; return; end

    ws = warning('off', 'MATLAB:polyshape:repairedBySimplify');
    cleaner = onCleanup(@() warning(ws)); %#ok<NASGU>

    % Build (optionally buffered) polyshapes for overlap testing
    ps = repmat(polyshape, n, 1);
    for i = 1:n
        ps(i) = polyshape(polys{i}(:, 1), polys{i}(:, 2), 'Simplify', true);
        if gap > 0
            ps(i) = polybuffer(ps(i), gap / 2);   % half-gap each -> total gap
        end
    end

    % Pairwise adjacency (includes the diagonal as true)
    if n == 1
        adj = true;
    else
        adj = overlaps(ps);
    end

    % Connected components via iterative BFS (no graph-toolbox dependency)
    seen   = false(n, 1);
    groups = {};
    for s = 1:n
        if seen(s), continue; end
        stack = s;  comp = [];
        while ~isempty(stack)
            v = stack(end);  stack(end) = [];
            if seen(v), continue; end
            seen(v) = true;  comp(end + 1) = v; %#ok<AGROW>
            nb = find(adj(v, :) & ~seen(:)');
            stack = [stack, nb]; %#ok<AGROW>
        end
        groups{end + 1} = sort(comp); %#ok<AGROW>
    end
end

% =========================================================================
function out = group_outline(polys)
% GROUP_OUTLINE  Real (un-fitted) outline of a group of footprints: the union
% of the member polygons. Single-member groups keep their exact footprint.
    if numel(polys) == 1
        out = polys{1};
        return
    end
    ws = warning('off', 'MATLAB:polyshape:repairedBySimplify');
    cleaner = onCleanup(@() warning(ws)); %#ok<NASGU>
    psu = polyshape(polys{1}(:, 1), polys{1}(:, 2), 'Simplify', true);
    for i = 2:numel(polys)
        psi = polyshape(polys{i}(:, 1), polys{i}(:, 2), 'Simplify', true);
        psu = union(psu, psi);
    end
    out = psu.Vertices;     % NaN-separated if the union has multiple regions
end
