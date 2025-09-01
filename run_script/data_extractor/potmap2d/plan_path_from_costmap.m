function [path, info] = plan_path_from_costmap(costmap_i8, start_rc, goal_rc, opts)
% A* on int8 costmap [-1,0..100] with 8-connectivity + Euclidean heuristic.
% -1: unknown, 0..100: occupancy/cost (0=free)

if nargin < 4, opts = struct(); end
occ_threshold     = getOpt(opts,'occ_threshold',     99);   % >= threshold = blocked
allow_unknown     = getOpt(opts,'allow_unknown',     true);
inflate_radius_px = getOpt(opts,'inflate_radius_px', 0);
occ_cost_weight   = getOpt(opts,'occ_cost_weight',   2.0);
unknown_cost      = getOpt(opts,'unknown_cost',      0.5);
snap_start        = getOpt(opts,'snap_start',        true);
snap_goal         = getOpt(opts,'snap_goal',         true);

assert(ismatrix(costmap_i8) && isa(costmap_i8,'int8'), 'costmap_i8 must be HxW int8');

[H,W] = size(costmap_i8);
cm  = double(costmap_i8);
unk = (cm == -1);
blk = (cm >= occ_threshold);
if ~allow_unknown, blk = blk | unk; end

% Inflate around BLOCKED cells
if inflate_radius_px > 0
    D = bwdist(blk);
    blk = blk | (D <= inflate_radius_px);
end
free = ~blk;

% Snap/validate endpoints
if snap_start, s = snapToFree(start_rc, free, H, W); else, s = validateOrEmpty(start_rc, free, H, W); end
if snap_goal,  g = snapToFree(goal_rc,  free, H, W); else, g = validateOrEmpty(goal_rc,  free, H, W); end
if isempty(s) || isempty(g), path = []; info = makeInfo([],[],[],[],free); return; end
sr = s(1); sc = s(2);  gr = g(1); gc = g(2);

% Per-cell traversal penalties (double)
cost_extra = zeros(H,W);
known = ~unk;
cost_extra(known) = max(cm(known),0)./100 * occ_cost_weight;
if allow_unknown, cost_extra(unk) = unknown_cost; end

% 8-connected moves
dirs = int8([ -1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1 ]);
stepCost = [1;1;1;1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];

% Euclidean heuristic to goal
heur = @(r,c) hypot(double(r - gr), double(c - gc));

% A*
startIdx = sub2ind([H W], sr, sc);
goalIdx  = sub2ind([H W], gr, gc);
gScore   = inf(H,W); gScore(startIdx) = 0;
fScore   = inf(H,W); fScore(startIdx) = heur(sr,sc);
closed   = false(H,W);
parentR  = zeros(H,W); parentC = zeros(H,W);
openSet  = startIdx;

while ~isempty(openSet)
    [~, k] = min(fScore(openSet));
    cur    = openSet(k); openSet(k) = [];
    [r,c]  = ind2sub([H W], cur);
    if closed(r,c), continue; end
    closed(r,c) = true;

    if cur == goalIdx
        path = [r c];
        while ~(r==sr && c==sc)
            pr = parentR(r,c); pc = parentC(r,c);
            r = pr; c = pc; path(end+1,:) = [r c]; %#ok<AGROW>
        end
        path = flipud(path);
        info = makeInfo([sr sc],[gr gc], ~isequal([sr sc],start_rc), ~isequal([gr gc],goal_rc), free);
        return
    end

    for d = 1:8
        nr = r + dirs(d,1); nc = c + dirs(d,2);
        if nr<1 || nr>H || nc<1 || nc>W || ~free(nr,nc) || closed(nr,nc), continue; end
        nIdx = sub2ind([H W], nr, nc);
        tentative_g = gScore(r,c) + stepCost(d) + cost_extra(nr,nc);
        if tentative_g + 1e-9 < gScore(nr,nc)
            gScore(nr,nc) = tentative_g;
            parentR(nr,nc) = r; parentC(nr,nc) = c;
            fScore(nr,nc)  = tentative_g + heur(nr,nc);
            if ~any(openSet == nIdx), openSet(end+1) = nIdx; end %#ok<AGROW>
        end
    end
end

path = [];
info = makeInfo([sr sc],[gr gc], ~isequal([sr sc],start_rc), ~isequal([gr gc],goal_rc), free);
end

% --------- helpers ---------
function val = getOpt(s, field, default)
if isfield(s, field), val = s.(field); else, val = default; end
end
function p = validateOrEmpty(p, free, H, W)
r = round(p(1)); c = round(p(2));
if r<1 || r>H || c<1 || c>W || ~free(r,c), p = []; else, p = [r c]; end
end
function p = snapToFree(p, free, H, W)
r = round(p(1)); c = round(p(2)); r = min(max(r,1),H); c = min(max(c,1),W);
if free(r,c), p = [r c]; return; end
[~, idx] = bwdist(free);
if isempty(idx), p = []; return; end
lin = idx(r,c); if lin==0, p = []; return; end
[rr, cc] = ind2sub(size(free), lin);
if free(rr,cc), p = [rr cc]; else, p = []; end
end
function info = makeInfo(s_used, g_used, snapped_s, snapped_g, free)
info = struct('start_used', s_used, 'goal_used', g_used, ...
              'snapped_start', logical(snapped_s), 'snapped_goal', logical(snapped_g), ...
              'free_mask', free);
end