
clear all; close all; clc

% read yaml
base_dir = fileparts( fileparts(pwd) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',base_dir);
config = ReadYaml(config_file) ;

inpath = config.navdata_extractor.inpath ;
base_sync_metadata_dir = '/media/mydata/former_datasets/former28' %config.navdata_extractor.outpath ;
% 
% str_split = split(inpath, '/') ;
% navtime_id = str_split{end} ;
% 
% 
% 
% mydir = dir(extraction_dir) ;
% 
% extracted_dir = mydir(1).folder ;
% metadata_dirs = dir( extracted_dir ) ;
% bag_id = metadata_dirs(3).name ;

inpath_splits = split(config.navdata_extractor.inpath, '/') ;
navtime_id = inpath_splits{end}  ;
extraction_dir = sprintf('%s/%s',base_sync_metadata_dir, navtime_id ) ;
processed_bags_dir = dir(extraction_dir) ;
procssed_bag_ids = {} ;
cnt = 1;
for idx=3:length(processed_bags_dir )
    procssed_bag_ids{cnt} = processed_bags_dir(idx).name ;
    cnt = cnt + 1;
end

%sync_metadata_dir = sprintf('%s/%s/%s/synced', base_sync_metadata_dir, navtime_id, procssed_bag_ids{1}) ;
%sync_metadata_dir = '/media/mydata/former_datasets/former28/bag_2025-05-30-14-14-01/synced'
%sync_metadata_dir = '/media/results/navdata_collector/deployments/processed/2025-08-03-15-47/bag_2025-08-03-15-47-55/synced'

odom = load(sprintf("%s/sync_odom.txt", sync_metadata_dir)) ;
[num_data, c] = size(odom) ;

px_min = min(odom(:,5)) ;
px_max = max(odom(:,5)) ;
py_min = min(odom(:,6)) ;
py_max = max(odom(:,6)) ;

tf_m2b = load(sprintf('%s/sync_tf_m2b.txt', sync_metadata_dir)) ;
tf_m2o = load(sprintf('%s/sync_tf_m2o.txt', sync_metadata_dir))
% odom: idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz 
% tf:   idx, 0,   time(s), time(ns), px, py, pz, qx, qy, qz, qw

fig = figure() ;
target_size = [1024, 1024] ;
fig.Position = [100 100 1024 1024]; %2048 2048];
t = tiledlayout("horizontal",'TileSpacing','Compact','Padding','Compact'); 
xy = zeros(num_data, 2) ;

vidfile = VideoWriter('/home/hankm/Desktop/bag_sample') ;
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


rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;

% normlize depth img
depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

% draw odom
pose7 = odom(1,5:11) ;
orient = quat2eul( pose7(4:end) ) ;
px = pose7(1);
py = pose7(2); 
theta = orient(3)  ;
wHr_prev = xyzypr_to_htm( [px,py,0,orient] ) ;

for idx=1 : num_data-1
    idx
    rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
    depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;
    
    % normlize depth img
    depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

    % draw odom
    pose7_odom = odom(idx,5:11) ; % x y z qx qy qz qw
    oHr = quat_to_htm( pose7_odom([end, 4:6]) ) ;
    oHr(1:2,4) = pose7_odom(1:2) ;
    [px_odom, py_odom, z, rol, pit, theta_odom] = htm_to_xyzypr( oHr ) ;
    xy_odom(idx,:) = [px_odom, py_odom] ;
    twist = odom(idx+1, 12:end) ;

    % draw m2b  (slam pose)
    pose7_m2b = tf_m2b(idx,5:11) ;
    htm = quat_to_htm( pose7_m2b([end, 4:6]) ) ;
    [px_m2b, py_m2b, z, rol, pit, theta_m2b] = htm_to_xyzypr( htm ) ;
    xy_m2b(idx,:) = [px_m2b, py_m2b] ;

    % m2o
    pose7_m2o = tf_m2o(idx,5:11) ;
    mHo = quat_to_htm( pose7_m2o([end, 4:6]) ) ;
    mHo(1:2,4) = pose7_m2o(1:2) ;
    [px_m2o, py_m2o, z, rol, pit, theta_m2o] = htm_to_xyzypr( mHo ) ;
    xy_m2o(idx,:) = [px_m2o, py_m2o] ;

    % m2r --> m2o2r
    mHr = mHo * oHr ;
    [px_m2r, py_m2r, z, rol, pit, yaw] = htm_to_xyzypr(mHr) ;
    mHr(1:2,4) = [px_m2r, py_m2r];
    theta_m2r = yaw  ;
    xy_m2r(idx,:) = [px_m2r, py_m2r] ;


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
    title('m2o2r pose and traj')

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

    wHr_prev = wHr ;
end

close(vidfile) ;




