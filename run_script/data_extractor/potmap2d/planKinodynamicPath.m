function poses = planKinodynamicPath(P0, P1, P2, v_max, w_max, FPS, opts)


    if nargin < 7 || isempty(opts), opts = struct; end
    if ~isfield(opts,'TangentGain'),  opts.TangentGain  = 0.6;   end
    if ~isfield(opts,'GridN'),        opts.GridN        = 1500;  end
    if ~isfield(opts,'MaxIter'),      opts.MaxIter      = 25;    end
    if ~isfield(opts,'UseP1Quat'),    opts.UseP1Quat    = false; end
    if ~isfield(opts,'KappaGeomMax'), opts.KappaGeomMax = inf;   end
    if ~isfield(opts,'Safety'),       opts.Safety       = 0.98;  end

    % ---------- unpack and choose P1 heading ----------
    [x0,y0,z0,th0] = unpackPose(P0);
    [x1,y1,z1,th1p]= unpackPose(P1);
    [x2,y2,z2,th2] = unpackPose(P2);
    if opts.UseP1Quat
        th1 = th1p;
    else
        th1 = atan2(y2 - y1, x2 - x1);   % heading at P1 points to P2
    end

    % chord lengths
    L01 = hypot(x1-x0, y1-y0);
    L12 = hypot(x2-x1, y2-y1);

    % degenerate
    if L01 < 1e-9 && L12 < 1e-9
        yaw = unwrap([th0; th2]);
        Q   = quatContinuous(yaw2quat_vec(yaw));
        poses = [repmat([x0 y0 z0],2,1), Q];
        return;
    end

    % ---------- endpoint tangents (meters) ----------
    k0 = opts.TangentGain * max(L01, 1e-9);
    k1 = opts.TangentGain * max(min(L01,L12), 1e-9);
    k2 = opts.TangentGain * max(L12, 1e-9);
    t0v = k0*[cos(th0); sin(th0)];
    t1v = k1*[cos(th1); sin(th1)];
    t2v = k2*[cos(th2); sin(th2)];

    % optional geometric curvature limit (independent of speed)
    kappa_geom_max = opts.KappaGeomMax * opts.Safety;
    if isfinite(kappa_geom_max)
        [t0,t1,t2] = solveSharedTangent(x0,y0,x1,y1,x2,y2, t0v,t1v,t2v, kappa_geom_max, opts.GridN, opts.MaxIter);
    else
        t0 = t0v; t1 = t1v; t2 = t2v;
    end

    % ---------- dense grids & arc-length ----------
    u01 = linspace(0,1, opts.GridN).';
    u12 = linspace(0,1, opts.GridN).';

    [x01,y01,dx01,dy01,ddx01,ddy01] = hermiteXY(u01, x0,y0,x1,y1, t0(1),t0(2), t1(1),t1(2));
    [x12,y12,dx12,dy12,ddx12,ddy12] = hermiteXY(u12, x1,y1,x2,y2, t1(1),t1(2), t2(1),t2(2));

    s01 = cumtrapz(u01, hypot(dx01,dy01));  S01 = s01(end);
    s12 = cumtrapz(u12, hypot(dx12,dy12));  S12 = s12(end);

    % concat (drop duplicate joint)
    X  = [x01; x12(2:end)];
    Y  = [y01; y12(2:end)];
    dX = [dx01; dx12(2:end)];
    dY = [dy01; dy12(2:end)];
    ddX= [ddx01; ddx12(2:end)];
    ddY= [ddy01; ddy12(2:end)];
    S  = [s01;   s12(2:end)+S01];          % arclength from 0..Stot
    Stot = S(end);

    % curvature along path
    num   = abs(dX.*ddY - dY.*ddX);
    den   = (dX.^2 + dY.^2).^(3/2) + 1e-15;
    kappa = num ./ den;

    % ---------- kinodynamic time-scaling (no accel limit) ----------
    % enforce both v<=v_max and |omega|<=w_max => v_cap(s) = min(v_max, w_max/|kappa|)
    v_cap = v_max * ones(size(kappa));
    mask  = (kappa > 1e-9);
    v_cap(mask) = min(v_cap(mask), w_max ./ kappa(mask));

    % build time mapping t(S) via trapezoid over ds/v_cap
    ds    = diff(S); ds(ds<=0) = 1e-12;
    %dtSeg = 2*ds ./ (v_cap(1:end-1) + v_cap(end:-1:2)); % robust average
    dtSeg = 2*ds ./ (v_cap(1:end-1) + v_cap(2:end));
    Tall  = [0; cumsum(dtSeg)];
    Ttot  = Tall(end);

% sample at fixed FPS in time, invert to arclength
% dt      = 1/FPS;
% t_samp  = 0:dt:Ttot; if t_samp(end) < Ttot-1e-12, t_samp(end+1) = Ttot; end
% s_samp  = interp1(Tall, S, t_samp, 'linear','extrap');

%num_points = ceil(Ttot * FPS);  % number of output poses
%s_samp = linspace(0, Stot, num_points);  % arc-length uniform sampling

% NEW: arc-length uniform sampling with ≥16 poses
base_n   = max(1, ceil(Ttot * FPS));
min_n    = 16;
target_n = max(min_n, base_n);

if Stot < 1e-12
    ds = 1e-3;
    s_samp = (0:target_n-1)' * ds;
else
    ds = Stot / max(target_n - 1, 1);
    s_samp = (0:target_n-1)' * ds;   % uniform arc-length
end


    % evaluate spline at s_samp
    xs = zeros(numel(s_samp),1); ys = xs; dxs = xs; dys = xs;
    for k = 1:numel(s_samp)
        if s_samp(k) <= S01
            u = interp1(s01, u01, s_samp(k), 'linear','extrap');
            [xk,yk,dxk,dyk] = hermiteXY(u, x0,y0,x1,y1, t0(1),t0(2), t1(1),t1(2));
        elseif s_samp(k) <= Stot
            s2 = s_samp(k) - S01;
            u  = interp1(s12, u12, s2, 'linear','extrap');
            [xk,yk,dxk,dyk] = hermiteXY(u, x1,y1,x2,y2, t1(1),t1(2), t2(1),t2(2));
        else
            % ---- extension past P2 along heading th2 ----
            dS  = s_samp(k) - Stot;
            xk  = x2 + dS * cos(th2);
            yk  = y2 + dS * sin(th2);
            dxk = cos(th2);
            dyk = sin(th2);
        end
        xs(k)=xk; ys(k)=yk; dxs(k)=dxk; dys(k)=dyk;
    end

    % headings (unwrap + anchor) & sign-continuous quats
    yaw = unwrap(atan2(dys, dxs)); yaw(1)=th0; yaw(end)=th2;
    z   = linspace(z0, z2, numel(yaw)).';
    Q   = quatContinuous(yaw2quat_vec(yaw));
    poses = [xs, ys, z, Q];
end

% ===================== helpers (same file) =====================

function [x,y,z,yaw] = unpackPose(P)
    x=P(1); y=P(2); z=P(3);
    qw=P(4); qx=P(5); qy=P(6); qz=P(7);
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
end

function Q = yaw2quat_vec(yaw)
    yaw = yaw(:); N = numel(yaw);
    Q   = [cos(yaw/2), zeros(N,1), zeros(N,1), sin(yaw/2)]; % qw, qx,qy,qz
end

function Qout = quatContinuous(Qin)
    Qout = Qin;
    for i = 2:size(Qout,1)
        if dot(Qout(i,:), Qout(i-1,:)) < 0
            Qout(i,:) = -Qout(i,:);
        end
    end
end

function [t0, t1, t2] = solveSharedTangent(...
    x0,y0,x1,y1,x2,y2, t0v,t1v,t2v, kappa_max, N, MaxIter)
% Scale t0,t1,t2 (with shared t1) so both segments respect kappa_max.
    lo1 = 0; hi1 = 1; s1 = 1; found=false; best=[0,0,0];
    for it = 1:MaxIter
        t1_try = s1 * t1v;
        s0 = findScaleForCurv(@(s0in) hermiteMaxCurv(x0,y0,x1,y1, s0in*t0v(1), s0in*t0v(2), t1_try(1), t1_try(2), N), kappa_max, MaxIter);
        s2 = findScaleForCurv(@(s2in) hermiteMaxCurv(x1,y1,x2,y2, t1_try(1), t1_try(2), s2in*t2v(1), s2in*t2v(2), N), kappa_max, MaxIter);
        k01 = hermiteMaxCurv(x0,y0,x1,y1, s0*t0v(1), s0*t0v(2), t1_try(1), t1_try(2), N);
        k12 = hermiteMaxCurv(x1,y1,x2,y2, t1_try(1), t1_try(2), s2*t2v(1), s2*t2v(2), N);
        if k01<=kappa_max && k12<=kappa_max, best=[s0,s1,s2]; found=true; lo1=s1; else, hi1=s1; end
        s1 = 0.5*(lo1+hi1); if hi1-lo1<1e-3, break; end
    end
    if ~found, best=[0,0,0]; end
    t0 = best(1)*t0v; t1 = best(2)*t1v; t2 = best(3)*t2v;
end

function s_best = findScaleForCurv(curvFunc, kappa_max, MaxIter)
    lo=0; hi=1; s=1;
    for it=1:MaxIter
        if curvFunc(s)>kappa_max, hi=s; else, lo=s; end
        s = 0.5*(lo+hi); if hi-lo<1e-3, break; end
    end
    s_best = lo;
end

function [kmax] = hermiteMaxCurv(xa,ya,xb,yb,txa,tya,txb,tyb,N)
    u = linspace(0,1,N).';
    [~,~,dx,dy,ddx,ddy] = hermiteXY(u, xa,ya,xb,yb, txa,tya, txb,tyb);
    num = abs(dx.*ddy - dy.*ddx);
    den = (dx.^2 + dy.^2).^(3/2) + 1e-15;
    kappa = num ./ den;
    kmax  = max(kappa);
end

function [x,y,dx,dy,ddx,ddy] = hermiteXY(u,x0,y0,x1,y1,tx0,ty0,tx1,ty1)
    u  = u(:); u2 = u.*u; u3 = u2.*u;
    H00 =  2*u3 - 3*u2 + 1;    H10 =    u3 - 2*u2 + u;
    H01 = -2*u3 + 3*u2;        H11 =    u3 -     u2;

    dH00 = 6*u2 - 6*u;         dH10 = 3*u2 - 4*u + 1;
    dH01 = -dH00;              dH11 = 3*u2 - 2*u;

    ddH00 = 12*u - 6;          ddH10 = 6*u - 4;
    ddH01 = -ddH00;            ddH11 = 6*u - 2;

    x   = H00*x0 + H10*tx0 + H01*x1 + H11*tx1;
    y   = H00*y0 + H10*ty0 + H01*y1 + H11*ty1;

    dx  = dH00*x0 + dH10*tx0 + dH01*x1 + dH11*tx1;
    dy  = dH00*y0 + dH10*ty0 + dH01*y1 + dH11*ty1;

    ddx = ddH00*x0 + ddH10*tx0 + ddH01*x1 + ddH11*tx1;
    ddy = ddH00*y0 + ddH10*ty0 + ddH01*y1 + ddH11*ty1;
end