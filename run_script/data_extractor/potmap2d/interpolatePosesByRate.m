function poses = interpolatePosesByRate(P0, P2, robot_speed, resolution, FPS)
% P0, P2: [x y z qw qx qy qz]
% robot_speed [m/s], resolution [m/cell] (not strictly needed), FPS [Hz]
% returns (N+1) x 7 poses from t=0..1 inclusive

    %--- distance per frame (meters) ---
    d_per_frame = robot_speed / resolution/ FPS;    % equals (resolution * cells_per_frame)
    if d_per_frame <= 0
        error('robot_speed and FPS must be positive');
    end

    %--- how many frames do we need? ---
    p0 = P0(1:3);  p2 = P2(1:3);
    dist = norm(p2 - p0);               % meters
    N = max(1, ceil(dist / d_per_frame));   % at least 1 step so we include both ends

    %--- sample t uniformly in time ---
    ts = linspace(0, 1, N+1);

    %--- interpolate each sample ---
    poses = zeros(numel(ts), 7);
    for i = 1:numel(ts)
        poses(i,:) = interpolatePoseSlerp(P0, P2, ts(i));
    end
end

function P = interpolatePoseSlerp(P0, P2, t)
% Interpolate one pose at fraction t in [0,1]
    p0 = P0(1:3);   p2 = P2(1:3);
    q0 = P0(4:7).'; q1 = P2(4:7).';

    % normalize and enforce shortest path
    q0 = q0 / norm(q0); q1 = q1 / norm(q1);
    if dot(q0,q1) < 0, q1 = -q1; end

    % SLERP
    c = max(min(dot(q0,q1),1),-1);
    if c > 0.9995
        q = (1-t)*q0 + t*q1; q = q/norm(q);
    else
        th = acos(c); s = sin(th);
        q = (sin((1-t)*th)/s)*q0 + (sin(t*th)/s)*q1;
    end

    % LERP position
    p = (1-t)*p0 + t*p2;

    P = [p(:).' q(:).'];
end