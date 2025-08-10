
% It shows (1) the topomap's tf1_m2b 
% (2) colldata's tf2_m2b and (3) odom

clear all; close all; clc;

tf1_data_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/round10-1/topomap' ;
tf2_data_dir = '/media/results/navdata_collector/collision_data/processed/coll_2025-08-10-15-05/bag_2025-08-10-15-05-17/synced'
tf1_m2b_file = sprintf('%s/topo_tf_m2b.txt', tf1_data_dir) ;
tf2_m2o_file = sprintf('%s/sync_tf_m2o.txt', tf2_data_dir) ;
tf2_o2b_file = sprintf('%s/sync_odom.txt', tf2_data_dir) ;
tf2_m2b_file = sprintf('%s/sync_tf_m2b.txt', tf2_data_dir) ;

[tf1_m2b_raw, tf1_xy_m2b, m1Hb] = load_pose_data( tf1_m2b_file ) ;
[tf2_m2o_raw, tf2_xy_m2o, m2Ho2] = load_pose_data( tf2_m2o_file ) ;
[tf2_o2b_raw, tf2_xy_o2b, o2Hb] = load_pose_data( tf2_o2b_file ) ;
[tf2_m2b_raw, tf2_xy_m2b, m2Hb] = load_pose_data( tf2_m2b_file ) ;


[num_topo] = size(tf1_m2b_raw) ;
[num_pose_data, ~] = size(tf2_xy_m2b) ;

for idx=1:num_topo
    m1Hb(:,:,idx) = inv(m1Hb(:,:,1)) * m1Hb(:,:,idx) ;
end

for idx=1:num_pose_data
    m2Hb(:,:,idx) = inv(m2Hb(:,:,1)) * m2Hb(:,:,idx) ;
end

tf1_xy_m2b = [squeeze(m1Hb(1,4,:))  squeeze(m1Hb(2,4,:)) ] ;
tf2_xy_m2b = [squeeze(m2Hb(1,4,:))  squeeze(m2Hb(2,4,:)) ] ;

% compute m2Hb
m2Hb_est = zeros(4,4,num_pose_data) ;
xy_m2_b = zeros(num_pose_data, 2) ;
for idx=1:num_pose_data
    m2Hb_est(:,:,idx) = m2Ho2(:,:,idx) * o2Hb(:,:,idx) ;
end

tf2_xy_m2b_est = [squeeze(m2Hb_est(1,4,:))  squeeze(m2Hb_est(2,4,:)) ] ;


plot(tf1_xy_m2b(:,1), tf1_xy_m2b(:,2), 'r*' ); hold on
plot(tf2_xy_m2b(:,1), tf2_xy_m2b(:,2), 'mo' )
plot(tf2_xy_m2b_est(:,1), tf2_xy_m2b_est(:,2), 'c.' )



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







