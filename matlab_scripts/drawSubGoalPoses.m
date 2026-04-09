function p = drawSubGoalPoses(sgs_xyzq_corrected_px, rx,  ry, vstep)

    sgs_px_cands = [sgs_xyzq_corrected_px(:,1) +  rx, sgs_xyzq_corrected_px(:,2) +  ry] ;
    quat = sgs_xyzq_corrected_px(:,4:end) ;
    yaw = atan2(2*(quat(:,1).*quat(:,4) + quat(:,2).*quat(:,3)), 1 - 2.*(quat(:,3).^2 + quat(:,4).^2));
    
    dirvec = [cos(yaw), sin(yaw)];
    scale = 8 ;
    
    u = sgs_px_cands(1:vstep:end,1) ;
    v = sgs_px_cands(1:vstep:end,2) ;
    dvx = scale* dirvec(1:vstep:end,1);
    dvy = scale* dirvec(1:vstep:end,2);
    
    p = plot( u, v, 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b' ) ;
    hold on;
    h = quiver(u, v, dvx, dvy, 0, 'LineWidth', 2, 'Color', 'r', 'MaxHeadSize', 10 )
    h.MaxHeadSize = 12 ;

end