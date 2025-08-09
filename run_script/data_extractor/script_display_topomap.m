

clear all; close all; clc

% read yaml
base_dir = fileparts( fileparts(pwd) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',base_dir);
config = ReadYaml(config_file) ;

inpath = config.navdata_extractor.inpath ;
topomap_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/round0/topomap';

odom = load(sprintf("%s/topo_odom.txt", topomap_dir)) ;
[num_data, c] = size(odom) ;

px_min = min(odom(:,5)) ;
px_max = max(odom(:,5)) ;
py_min = min(odom(:,6)) ;
py_max = max(odom(:,6)) ;

tf_m2b = load(sprintf('%s/topo_tf_m2b.txt', topomap_dir)) ;
tf_m2o = load(sprintf('%s/topo_tf_m2o.txt', topomap_dir)) ;
% odom: idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz 
% tf:   idx, 0,   time(s), time(ns), px, py, pz, qx, qy, qz, qw

fig = figure() ;
target_size = [1024, 1024] ;
fig.Position = [100 100 1024 1024]; %2048 2048];
t = tiledlayout("horizontal",'TileSpacing','Compact','Padding','Compact'); 
xy = zeros(num_data, 2) ;

vidfile = VideoWriter('/home/hankm/Desktop/topomap') ;
vidfile.FrameRate = 40;
open(vidfile) ;

% init pose
px0 = odom(1,5) ;
py0 = odom(1,6) ;
orient = quat2eul( odom(1, 8: 11) ) ; 
theta0 = orient(3)  ;
[hx0, hy0] = pol2cart(theta0, 1) ;

euc_dist = zeros(1,length(odom)) ;
ang_dist = zeros(1,length(odom)) ;

% draw odom
pose7 = odom(1,5:11) ;
rpy_odom = quat2eul( pose7(4:end) ) ;
px_odom = pose7(1);
py_odom = pose7(2); 
theta_odom = rpy_odom(3)  ;
wHr_prev = xyzypr_to_htm( [px_odom,py_odom,0,rpy_odom] ) ;

for idx=1 : num_data-1
    fidx = (idx - 1) * 10
    rgbimg = imread( sprintf('%s/rgb%05d.png',topomap_dir, fidx) ) ;
    depthimg = imread(sprintf('%s/depth%05d.png',topomap_dir, fidx) )  ;
    
    % normlize depth img
    depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

    % draw odom
    pose7_odom = odom(idx,5:11) ; % x y z qx qy qz qw
    oHr = quat_to_htm( pose7_odom([end, 4:6]) ) ;
    oHr(1:2,4) = pose7_odom(1:2) ;
    [px_odom, py_odom, ~, rol_odom, pit_odom, theta_odom] = htm_to_xyzypr( oHr ) ;
    xy_odom(idx,:) = [px_odom, py_odom] ;
    twist = odom(idx+1, 12:end) ;

    % draw m2b  (slam pose)
    pose7_m2b = tf_m2b(idx,5:11) ;
    htm = quat_to_htm( pose7_m2b([end, 4:6]) ) ;
    [px_m2b, py_m2b, ~, rol_m2b, pit_m2b, theta_m2b] = htm_to_xyzypr( htm ) ;
    xy_m2b(idx,:) = [px_m2b, py_m2b] ;

    % m2o
    pose7_m2o = tf_m2o(idx,5:11) ;
    mHo = quat_to_htm( pose7_m2o([end, 4:6]) ) ;
    mHo(1:2,4) = pose7_m2o(1:2) ;
    [px_m2o, py_m2o, ~, rol_m2o, pit_m2o, theta_m2o] = htm_to_xyzypr( mHo ) ;
    xy_m2o(idx,:) = [px_m2o, py_m2o] ;

    % m2r --> m2o2r
    mHr = mHo * oHr ;
    [px_m2r, py_m2r, ~, rol_m2r, pit_m2r, yaw_m2r] = htm_to_xyzypr(mHr) ;
    mHr(1:2,4) = [px_m2r, py_m2r];
    theta_m2r = yaw_m2r  ;
    xy_m2r(idx,:) = [px_m2r, py_m2r] ;
end

%%========================================================================%%

fig; clf;
nexttile
imshow(rgbimg) ; title('RGB') ;

nexttile
%    imshow(depthimg / 255) ; title('Depth raw (0~255 recaled)') ;
imshow(depthimg_enhanced) ; title('Depth enhanced') ;

nexttile
plot(xy_odom(1:idx,1), xy_odom(1:idx,2), 'c.') ; hold on;
plot(px_odom, py_odom, 'oc', 'markersize', 10, 'MarkerFaceColor','c' ) ;
[hx_odom, hy_odom] = pol2cart(theta_odom, 1) ;
quiver( px0, py0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',8) ;
quiver( px_odom, py_odom, hx_odom, hy_odom,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ;
%quiver( px_odom, py_odom, hx_odom, hy_odom,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ; 

grid on; axis equal;  axis([px_min-11 px_max+11 py_min-1 py_max+1]); hold off;
title('Odom pose and traj')

% nexttile
% 
% plot(xy_m2b(1:idx,1), xy_m2b(1:idx,2), '.', 'Color', [1, 165/255, 0] ) ;  hold on;
% plot(px_m2b, py_m2b, 'ms', 'markersize', 8, 'MarkerFaceColor','m' ) ;
% [hx_m2b, hy_m2b] = pol2cart(theta_m2b, 1) ;
% quiver( px0, py0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',8) ;
% quiver( px_m2b, py_m2b, hx_m2b, hy_m2b,'AutoScale','off', 'Color', [1,0,1], 'LineWidth',2, 'MaxHeadSize',8) ;
% 
% grid on; axis equal;  axis([px_min-11 px_max+11 py_min-1 py_max+1]); hold off;
% title('SLAM pose and traj')
% 
% nexttile 

nexttile
plot(xy_m2r(1:idx,1), xy_m2r(1:idx,2), '.', 'Color', [1, 165/255, 0] ) ;  hold on;
plot(px_m2r, py_m2r, 'ms', 'markersize', 8, 'MarkerFaceColor','m' ) ;
[hx_m2r, hy_m2r] = pol2cart(theta_m2r, 1) ;
quiver( px0, py0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',8) ;
quiver( px_m2r, py_m2r, hx_m2r, hy_m2r,'AutoScale','off', 'Color', [1,0,1], 'LineWidth',2, 'MaxHeadSize',8) ;
grid on; axis equal;  axis([px_min-11 px_max+11 py_min-1 py_max+1]); hold off;
title('m2o x o2r pose and traj')

% nexttile
% [vx, vy] = pol2cart(twist(end), abs(twist(end))) ;
% plot(px, py, 'oc', 'markersize', 10, 'MarkerFaceColor','c' ) ; hold on;
% quiver( px, py, hx, hy,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ; 
% quiver( px, py, vx, vy,'AutoScale','off', 'Color', [1,0,1], 'LineWidth',4, 'MaxHeadSize',8) ; 
% grid on; axis equal;  axis([px-3 px+3 py-3 py+3]); hold off ;
% title('Heading Dir') ;

drawnow ;
set(gcf,'Position',[100 100 1024 1024])
F(idx) = getframe(gcf) ;

rgb_frame = frame2im(F(idx)) ;
resized_frame = imresize(rgb_frame, target_size) ;
writeVideo(vidfile, resized_frame) ;
pause (0.005) 

wHr = xyzypr_to_htm([px, py, 0, orient]) ;

if idx > 1
    r0Hr1 = inv(wHr_prev) * wHr ; 
    [dx, dy, dz, droll, dpit, dyaw] = htm_to_xyzypr( r0Hr1 ) ;
    euc_dist(idx) = sqrt(dx^2 + dy^2) ;
    ang_dist(idx) = dyaw ;
end









data_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/round10/synced' ;
odom_file = sprintf('%s/sync_odom.txt', data_dir) ;
m2o_file = sprintf('%s/sync_tf_m2o.txt', data_dir) ;

% idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz 
odom = load(odom_file) ;

% idx, 0,   time(s), time(ns), px, py, pz, qx, qy, qz, qw 
m2o = load(m2o_file) ;

[num_data, ~] = size(m2o) ;

xy_odom= zeros(num_data, 2) ;
xy_slam= zeros(num_data, 2) ;

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




