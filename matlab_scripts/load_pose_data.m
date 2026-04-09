function [pose_raw, xy, wHb] = load_pose_data( pose_file )

% load odom, tf_m2b, tf_m2b data
% where odm consists of : idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
% and tf_m2b condists of: idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw

    pose_raw = load(pose_file) ;
    [num_data, ~] = size(pose_raw) ;

    xy0 = pose_raw(1, 5:6) ;
    quat0 = pose_raw(1, [11,8:10]) ;
    
    wHb = zeros(4,4,num_data) ;
    wHb(:,:,1) = quat_to_htm( quat0 ) ;
    wHb(1:2,4,1) = xy0 ;

    xy = zeros(num_data, 2); 
    xy(1,:) = xy0 ;
    for idx=2:num_data
        xy_line = pose_raw(idx, 5:6) ;
        quat = pose_raw(idx, [11,8:10]) ;
        wHb(:,:,idx) = quat_to_htm( quat ) ;
        wHb(1:2,4,idx) = xy_line ;
        xy(idx,:) = xy_line ;
    end

end

