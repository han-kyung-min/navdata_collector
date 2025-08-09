
% It shows (1) the topomap's tf1_m2b 
% (2) colldata's tf2_m2b and (3) odom

tf1_data_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/round10/topomap' ;
tf2_data_dir = '/media/results/navdata_collector/collision_data/processed/coll_2025-07-13-17-41/bag_2025-07-13-17-41-23/synced' ;

tf1_m2b_file = sprintf('%s/topo_tf_m2b.txt', tf1_data_dir) ;
tf2_m2o_file = sprintf('%s/sync_tf_m2o.txt', tf2_data_dir) ;
tf2_o2b_file = sprintf('%s/sync_odom.txt', tf2_data_dir) ;

[tf1_m2b_raw, xy_m1_b, m1Hb] = load_pose_data( tf1_m2b_file ) ;
[tf2_m2o_raw, xy_m2_o2, m2Ho2] = load_pose_data( tf2_m2o_file ) ;
[tf2_o2b_raw, xy_o2_b, o2Hb] = load_pose_data( tf2_o2b_file ) ;

[num_topo] = size(tf1_m2b_raw) ;
[num_pose_data, ~] = size(m2o) ;

% compute m2Hb
m2Hb = zeros(4,4,num_pose_data) ;
xy_m2_b = zeros(num_pose_data, 2) ;
for idx=1:num_pose_data
    m2Hb(:,:,idx) = m2Ho2(:,:,idx) * o2Hb(:,:,idx) ;
    xy_m2_b(idx,:) = m2Hb(1:2,4,idx) ;
end

plot(xy_m1_b(:,1), xy_m1_b(:,2), 'r*' )
hold on
plot(xy_o2_b(:,1), xy_o2_b(:,2), 'b.' ) ; hold on
plot(xy_m2_b(:,1), xy_m2_b(:,2), 'm.' ) ;


for idx=1:num_data
    
    xr = odom(idx,5) ;
    yr = odom(idx,6) ;
    qx_r = odom(idx,8) ;
    qy_r = odom(idx,9) ;
    qz_r = odom(idx,10) ;
    qw_r = odom(idx,11) ;

    oHr(:,:,idx) = quat_to_htm( [qw_r, qx_r, qy_r, qz_r] ) ;
    oHr(1:2,4,idx) = [xr, yr] ;

    xo = m2o(idx,5) ;
    yo = m2o(idx,6) ;
    qx_o = m2o(idx,8) ;
    qy_o = m2o(idx,9) ;
    qz_o = m2o(idx,10) ;
    qw_o = m2o(idx,11) ;

    mHo(:,:,idx) = quat_to_htm( [ qw_o, qx_o, qy_o, qz_o] ) ;
    mHo(1:2,4,idx) = [xo, yo] ;
    mHr(:,:,idx) = mHo(:,:,idx) * oHr(:,:,idx) ;

    xy_odom(idx,:) = [xr, yr] ;
    xy_slam(idx,:) = mHr(1:2,4,idx) ;
end



s = 10 ;
plot(xy_odom(1:s:end,1), xy_odom(1:s:end,2), 'c.') ;  hold on ;
plot(xy_slam(1:s:end,1), xy_slam(1:s:end,2), 'r.') ;
legend('odom pose', 'SLAM pose')
axis equal







