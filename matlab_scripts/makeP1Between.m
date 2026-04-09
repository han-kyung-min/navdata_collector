function P1 = makeP1Between(P0, P2, varargin)
% makeP1Between  Point on the line P0->P2 with heading from P0 to P2.
%   P1 = makeP1Between(P0, P2)               % midpoint (Alpha=0.5)
%   P1 = makeP1Between(P0, P2, 'Alpha', a)   % fraction a in [0,1] from P0
%   P1 = makeP1Between(P0, P2, 'Distance', d)% d meters from P0 toward P2
%
% Inputs:
%   P0, P2 : 1x7 poses [x y z qw qx qy qz] (quaternion is [w x y z])
% Options:
%   'Alpha'    : fraction along segment (default 0.5)
%   'Distance' : distance from P0 (overrides Alpha)
%
% Output:
%   P1 : 1x7 pose on the line with yaw pointing from P0 to P2
%
% Example:
%   P0 = [0 0 0 1 0 0 0];
%   P2 = [0.3255 -0.6178 0 0.9991 0 0 -0.0420];
%   P1 = makeP1Between(P0, P2, 'Alpha', 0.5);

    % --- parse options ---
    alpha = 0.5;   % default midpoint
    dist  = [];    % meters from P0 (optional)
    for k = 1:2:numel(varargin)
        key = lower(string(varargin{k}));
        val = varargin{k+1};
        switch key
            case {"alpha","t","fraction"}
                alpha = double(val);
            case {"distance","d"}
                dist = double(val);
            otherwise
                error('Unknown option "%s". Use ''Alpha'' or ''Distance''.', key);
        end
    end

    % --- positions & direction ---
    p0 = P0(1:3);
    p2 = P2(1:3);
    d  = p2 - p0;
    Lxy = hypot(d(1), d(2));

    % If Distance specified, convert to Alpha
    if ~isempty(dist)
        if Lxy < 1e-12
            alpha = 0;  % degenerate: points coincide in XY
        else
            alpha = dist / Lxy;
        end
    end
    alpha = max(0, min(1, alpha));  % clamp

    % --- position of P1 (linear interp in xyz) ---
    p1 = (1 - alpha) * p0 + alpha * p2;

    % --- heading: yaw from P0 -> P2 (fallback to P0's yaw if degenerate) ---
    if Lxy >= 1e-12
        th = atan2(d(2), d(1));
    else
        % yaw from P0 quaternion [qw qx qy qz] using ZYX convention
        qw = P0(4); qx = P0(5); qy = P0(6); qz = P0(7);
        th = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    end

    % --- quaternion for yaw about +Z (qw qx qy qz) ---
    q1 = [cos(th/2), 0, 0, sin(th/2)];

    % --- pack output ---
    P1 = [p1(:).', q1];
end