
function [out_waypts, target_waypoint] = sample_corrected_waypoints(P0, P2, resolution, fps, ws)

    if nargin < 5
        ws = 3;
    end
    robot_speed = 0.3 ; % 0.3 m/s
    dist_per_frame = robot_speed / resolution / fps;  % 6 px per a sec, 6/10 px per a frame  

    P1 = (P0 + P2)/2 + [0, 0];  % midpoint with offset = control point
    % Generate curve
    t = linspace(0, 1, 100);
    B = (1-t).^2'*P0 + 2*(1-t)'.*t'*P1 + t.^2'*P2;

    % Compute arc lengths along B
    dx = diff(B(:,1));
    dy = diff(B(:,2));
    segment_lengths = sqrt(dx.^2 + dy.^2);
    arc_lengths = [0; cumsum(segment_lengths)] ;

    % Find nearest point to current position
    dists_to_current = vecnorm(B - P0, 2, 2) ;
    [~, start_idx] = min(dists_to_current) ;
    start_arc = arc_lengths(start_idx) ;

    % Future distances
    num_wpts = 5 ;
    target_dists = start_arc + dist_per_frame * ws * (1:num_wpts) ;
    %target_dists < arc_lengths(end)  ;
    % Interpolate waypoints
    out_waypts = zeros(num_wpts, 2) ;
    valcnt = 0;
    for i = 1:num_wpts
        d = target_dists(i) ;
        if d > arc_lengths(end)
            out_waypts(i,:) = B(end,:);
        else
            idx = find(arc_lengths >= d, 1, 'first') ;
            if arc_lengths(idx) == d
                out_waypts(i,:) = B(idx,:);
            else
                % Linear interpolation
                d1 = arc_lengths(idx-1) ;
                d2 = arc_lengths(idx) ;
                w = (d - d1) / (d2 - d1) ;
                p1 = B(idx-1,:);
                p2 = B(idx,:);
                out_waypts(i,:) = (1 - w) * p1 + w * p2 ;
                valcnt = valcnt + 1;
            end
        end
    end

    if valcnt == 5
        out_waypts = out_waypts - P0 ;  % w.r.t P0 
        target_waypoint = out_waypts(3,:) ;
    else
        target_waypoint = P2 - P0;
        out_waypts = repmat( target_waypoint, [num_wpts,1] ) ;
    end

    orientations = zeros(num_wpts, 2);
    for i = 1:num_wpts-1
        dx = out_waypts(i+1, 1) - out_waypts(i, 1);   % wpt1  should heading to wpt2
        dy = out_waypts(i+1, 2) - out_waypts(i, 2);
        theta = atan2(dy, dx);  % radians
        half_theta = theta / 2 ;
        q = [cos(half_theta), zeros(size(theta)), zeros(size(theta)), sin(half_theta)] ;
        q = q / norm(q) ;
        orientations(i,:) = [q(1), q(4)] ;
    end
    orientations(end,:) = orientations(end-1,:) ;
    out_waypts = [ out_waypts, orientations ] ;

end