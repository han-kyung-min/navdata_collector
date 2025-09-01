function drawPoses2D(poses, varargin)
% poses: Nx7 [x y z qw qx qy qz]  OR  Nx3 [x y theta]
% options: 'ArrowLen' (m), 'Color', 'LineWidth'
    p = inputParser;
    addParameter(p,'ArrowLen',1.0);   % meters
    addParameter(p,'Color','m');
    addParameter(p,'LineWidth',1.0);
    parse(p,varargin{:});
    L = p.Results.ArrowLen;

    if size(poses,2)==7
        % quaternion -> yaw (ZYX)
        qw = poses(:,4); qx = poses(:,5); qy = poses(:,6); qz = poses(:,7);
        yaw = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
        x = poses(:,1); y = poses(:,2);
    elseif size(poses,2)==3
        x = poses(:,1); y = poses(:,2); yaw = poses(:,3);
    else
        error('poses must be Nx7 or Nx3');
    end

    u = L*cos(yaw);    % x-component of arrow
    v = L*sin(yaw);    % y-component of arrow

    hold on;
    % do not autoscale (use physical length L)
    quiver(x, y, u, v, 0, 'Color', p.Results.Color, ...
        'LineWidth', p.Results.LineWidth, 'MaxHeadSize', 2.0);
    plot(x, y, '.', 'Color', p.Results.Color);  % mark positions

    % quiver(x(3), y(3), u(3)*8, v(3)*8, 0, 'Color', 'r', ...
    % 'LineWidth', p.Results.LineWidth, 'MaxHeadSize', 2.0);
end