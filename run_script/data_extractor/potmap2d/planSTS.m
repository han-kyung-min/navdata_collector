function poses = planSTS(P0, P2, v_max, w_max, FPS)
% Kinematically-feasible Spin-Translate-Spin for differential drive
% P0,P2: [x y z qw qx qy qz]
% v_max [m/s], w_max [rad/s], FPS [Hz]
% returns N x 7 poses [x y z qw qx qy qz], sampled at dt=1/FPS

    dt = 1/FPS;

    %--- unpack and get yaw ---
    z0 = P0(3);  z2 = P2(3);              % keep z (usually 0)
    [x0,y0,th0] = unpack(P0);
    [x2,y2,th2] = unpack(P2);

    %--- heading of straight leg ---
    dx = x2 - x0;  dy = y2 - y0;
    L  = hypot(dx,dy);
    if L < 1e-9
        % just spin in place from th0 to th2
        ys = spinSegment(th0, th2, w_max, dt);
        poses = packConstPos(x0,y0,z0, ys);
        poses(end,:) = [x2 y2 z2 yaw2quat(th2)];  % ensure exact end
        return;
    end
    gamma = atan2(dy,dx);                          % travel direction

    %--- Segment A: spin th0 -> gamma ---
    yawA = spinSegment(th0, gamma, w_max, dt);

    %--- Segment B: translate along gamma at v_max ---
    T2   = L / max(v_max,eps);
    N2   = max(1, ceil(T2/dt));
    s    = linspace(0, L, N2+1);                   % arc length
    xB   = x0 + s.*cos(gamma);
    yB   = y0 + s.*sin(gamma);
    yawB = repmat(gamma, 1, numel(s));

    %--- Segment C: spin gamma -> th2 at goal position ---
    yawC = spinSegment(gamma, th2, w_max, dt);
    xC   = repmat(x2, 1, numel(yawC));
    yC   = repmat(y2, 1, numel(yawC));

    %--- stitch (avoid duplicate joints) ---
    x = [repmat(x0,1,numel(yawA)), xB(2:end), xC(2:end)];
    y = [repmat(y0,1,numel(yawA)) ,yB(2:end), yC(2:end)];
    yaw = [yawA, yawB(2:end), yawC(2:end)];

    %--- pack to [x y z qw qx qy qz] ---
    poses = zeros(numel(yaw),7);
    for i=1:numel(yaw)
        poses(i,:) = [x(i) y(i) z0 yaw2quat(yaw(i))];
    end
    % ensure exact final (no drift)
    poses(end,:) = [x2 y2 z2 yaw2quat(th2)];
end

% ---------- helpers ----------
function [x,y,yaw] = unpack(P)
    x=P(1); y=P(2);
    qw=P(4); qx=P(5); qy=P(6); qz=P(7);
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)); % ZYX
end

function q = yaw2quat(yaw)
    q = [cos(yaw/2), 0, 0, sin(yaw/2)]; % [qw qx qy qz]
end

function yaw = spinSegment(y0, y1, w_max, dt)
    d = wrapToPi(y1 - y0);
    T = abs(d)/max(w_max,eps);
    N = max(1, ceil(T/dt));
    alphas = linspace(0, 1, N+1);
    yaw = wrapToPi(y0 + alphas*d);
end

function P = packConstPos(x,y,z, yaw)
    P = zeros(numel(yaw),7);
    for i=1:numel(yaw)
        P(i,:) = [x y z yaw2quat(yaw(i))];
    end
end

function a = wrapToPi(a), a = mod(a+pi, 2*pi) - pi; end