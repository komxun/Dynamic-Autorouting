function [Bt, keep] = trim_buildings_corridor(B, p0, p1, margin)
% TRIM_BUILDINGS_CORRIDOR  Keep only buildings near the start->finish line.
%
%   [Bt, keep] = trim_buildings_corridor(B, p0, p1, margin)
%
%   Returns the subset of building boxes B whose footprint lies within
%   `margin` metres of the straight line segment from p0 to p1 (evaluated
%   in the horizontal X-Y plane). A building is kept when the distance from
%   its centre to the segment, minus the box's circumscribed footprint
%   radius, is no greater than `margin`. This restricts the IFDS
%   computation to the relevant flight corridor.
%
%   Inputs:
%     B      - struct array of building boxes (fields x0,y0,a,b,...)
%     p0,p1  - [x y] (or [x y z]) start and finish points
%     margin - [m] corridor half-width measured from the centre line
%
%   Outputs:
%     Bt     - trimmed struct array (buildings inside the corridor)
%     keep   - logical mask into B

    a = p0(1:2); a = a(:)';
    b = p1(1:2); b = b(:)';

    n = numel(B);
    keep = false(n, 1);
    for i = 1:n
        c   = [B(i).x0, B(i).y0];
        d   = point_seg_dist(c, a, b);
        rad = hypot(B(i).a, B(i).b);   % circumscribed radius of the box footprint
        if d - rad <= margin
            keep(i) = true;
        end
    end
    Bt = B(keep);
end

% =========================================================================
function d = point_seg_dist(p, a, b)
% Shortest distance from point p to segment a-b (2-D).
    ab = b - a;
    ap = p - a;
    denom = dot(ab, ab);
    if denom > 0
        t = max(0, min(1, dot(ap, ab) / denom));
    else
        t = 0;   % degenerate segment (a == b)
    end
    proj = a + t * ab;
    d = norm(p - proj);
end
