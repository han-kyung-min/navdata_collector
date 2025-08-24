
% It shows (1) the topomap's tf1_m2b 
% (2) colldata's tf2_m2b and (3) odom

tf1_data_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/round10-1/topomap' ;
tf2_data_dir = '/media/results/navdata_collector/collision_data/processed/coll_2025-08-21-18-33/bag_2025-08-21-18-33-42/synced' ;

tf1_m2b_file = sprintf('%s/topo_tf_m2b.txt', tf1_data_dir) ;
tf2_m2o_file = sprintf('%s/sync_tf_m2o.txt', tf2_data_dir) ;
tf2_o2b_file = sprintf('%s/sync_odom.txt', tf2_data_dir) ;

[tf1_m2b_raw, xy_m1_b, m1Hb] = load_pose_data( tf1_m2b_file ) ;
[tf2_m2o_raw, xy_m2_o2, m2Ho2] = load_pose_data( tf2_m2o_file ) ;
[tf2_o2b_raw, xy_o2_b, o2Hb] = load_pose_data( tf2_o2b_file ) ;

[num_topo] = size(tf1_m2b_raw) ;
[num_pose_data, ~] = size(tf2_o2b_raw) ;

% compute m2Hb
m2Hb = zeros(4,4,num_pose_data) ;
xy_m2_b = zeros(num_pose_data, 2) ;
for idx=1:num_pose_data
    m2Hb(:,:,idx) = m2Ho2(:,:,idx) * o2Hb(:,:,idx) ;
    xy_m2_b(idx,:) = m2Hb(1:2,4,idx) ;
end

figure(1); clf; 
% plot(xy_m1_b(:,1), xy_m1_b(:,2), 'rs' )
% hold on
% plot(xy_m2_b(:,1), xy_m2_b(:,2), 'g.' ) ;
% plot(xy_o2_b(:,1), xy_o2_b(:,2), 'c.' ) ;

for idx=1:num_pose_data   
    xy_odom_tf2(idx,:) = xy_o2_b(idx,:) ;
    xy_slam_tf2(idx,:) = m2Hb(1:2,4,idx) ;
end

for idx=1:num_topo
    xy_slam_tf1(idx,:) = m1Hb(1:2,4,idx) ;
end


s = 10 ;
plot(xy_slam_tf1(:,1), xy_slam_tf1(:,2), 'r*') ;  hold on ;
plot(xy_slam_tf2(1:s:end,1), xy_slam_tf2(1:s:end,2), 'c.') ;
plot(xy_odom_tf2(1:s:end,1), xy_odom_tf2(1:s:end,2), 'm.') ;

legend('SLAM pose', 'corrected odom pose', 'odom pose')
axis equal







