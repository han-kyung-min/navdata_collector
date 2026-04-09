function drawPoseSeq(varargin)

    pts_xyzq_corrected_px = varargin{1};
    rx = varargin{2} ;
    ry = varargin{3} ;
    vstep = varargin{4}; 

    if nargin < 5
        pose_marker= 'bo' ;
        fcolor='b' ;
    else
        pose_marker = varargin{5} ;
        fcolor = varargin{6} ;
    end

    pts_px_cands = [pts_xyzq_corrected_px(:,1) +  rx, pts_xyzq_corrected_px(:,2) +  ry] ;
    quat = pts_xyzq_corrected_px(:,4:end) ;
    yaw = atan2(2*(quat(:,1).*quat(:,4) + quat(:,2).*quat(:,3)), 1 - 2.*(quat(:,3).^2 + quat(:,4).^2));
    
    dirvec = [cos(yaw), sin(yaw)];
    scale = 8 ;
    
    u = pts_px_cands(1:vstep:end,1) ;
    v = pts_px_cands(1:vstep:end,2) ;
    dvx = scale* dirvec(1:vstep:end,1);
    dvy = scale* dirvec(1:vstep:end,2);
    
    plot( u, v, pose_marker, 'MarkerSize', 8, 'MarkerFaceColor', fcolor ) ;
    hold on;
    h = quiver(u, v, dvx, dvy, 0, 'LineWidth', 1, 'Color', 'r', 'MaxHeadSize', 10 )
    h.MaxHeadSize = 8 ;
end