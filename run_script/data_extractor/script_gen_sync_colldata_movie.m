close all;
clear all;
clc;

% read yaml
base_dir = fileparts( fileparts(pwd) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',base_dir);
config = ReadYaml(config_file) ;

inpath = config.navdata_extractor.inpath ;
outpath = config.navdata_extractor.outpath ;
%base_sync_metadata_dir = '/media/mydata/former_datasets/former28' %config.navdata_extractor.outpath ;

inpath_splits = split(config.navdata_extractor.inpath, '/') ;
navtime_id = inpath_splits{end}  ;
extraction_dir = sprintf('%s/%s',outpath, navtime_id ) ;
processed_bags_dir = dir(extraction_dir) ;
procssed_bag_ids = {} ;
cnt = 1;
for idx=3:length(processed_bags_dir )
    procssed_bag_ids{cnt} = processed_bags_dir(idx).name ;
    cnt = cnt + 1;
end

sync_metadata_dir = sprintf('%s/%s/synced', extraction_dir, procssed_bag_ids{1}) ;

% load odom
% odom: idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz 

odom_ = load(sprintf("%s/sync_odom.txt",sync_metadata_dir)) ;
[num_data, c] = size(odom_) ;

% tform odom such that it starts from (0,0)
odom0 = odom_(1,5:11) ;
wHb0 = quat_to_htm( odom0( [7,4:6]) ) ;
wHb0(1:3,4) = odom0(1:3) ;
b0Hw = inv(wHb0) ;

for idx=1:num_data
    odom_vec = odom_(idx, 5:11 );
    q = odom_vec([7,4:6])  ;
    wHbi = quat_to_htm( q ) ;
    wHbi(1:3,4) = odom_(idx, 5:7) ;
    b0Hbi(:,:,idx) = b0Hw * wHbi ;
    [x, y, z, rol, pit, yaw] = htm_to_xyzypr( b0Hbi(:,:,idx) ) ;
    odom(idx,:) = [x, y, z, rol, pit, yaw] ;
end

px_min = min(odom(:,1)) ;
px_max = max(odom(:,1)) ;
py_min = min(odom(:,2)) ;
py_max = max(odom(:,2)) ;

% load subgoals (relative)
% idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw 
sg = load( sprintf("%s/sync_rel_subgoals.txt", sync_metadata_dir)) ;

% load waypoints and dist to sgs
% idx, seq, time(s), time(ns), is_joy_on, xy_dist, orient_dist, px1, py1, qw1, qz2, px2, py2, qw2, qz2, ...
waypoint_data = load( sprintf('%s/sync_waypoints.txt', sync_metadata_dir) ) ;

% load joy
joy = load( sprintf('%s/sync_joy.txt', sync_metadata_dir));

fig = figure() ;
fig.Position = [100 100 1024 1024]; %2048 2048];
t = tiledlayout("horizontal",'TileSpacing','Compact','Padding','Compact'); 

xy = zeros(num_data, 2) ;

vidfile = VideoWriter('/home/hankm/Desktop/bag_colldata') ;
vidfile.FrameRate = 10;
open(vidfile) ;

% init pose
px0 = odom(1,1) ;
py0 = odom(1,2) ;
orient = odom(1, 4: end) ; 
theta0 = orient(3)  ;
[hx0, hy0] = pol2cart(theta0, 1) ;

euc_dist = zeros(1,length(odom)) ;
ang_dist = zeros(1,length(odom)) ;

rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;

% normlize depth img
depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

% draw odom
wHr_prev = xyzypr_to_htm( [px0, py0, 0, 0, 0, theta0] ) ;

for idx=2800 : num_data-1
    idx
    rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
    depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;
    scandata = load(sprintf('%s/scan%05d.txt', sync_metadata_dir, idx-1) ) ;
    scan_range = scandata(:,2) ;
    scan_angle = scandata(:,1) ;

    % normlize depth img
    depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

    % draw odom
    pose6 = odom(idx, 1:end) ;
    orient = pose6(4:end) ;
    px = pose6(1);
    py = pose6(2); 
    theta = orient(3)  ;

    xy(idx,:) = [px, py] ;

    % draw sg
    xy_sg = sg(idx, 5:6) ;
    
    % draw waypoint data
    is_joy_on = waypoint_data(idx, 5) ;
    xy_dist = waypoint_data(idx, 6) ;
    orient_dist = waypoint_data(idx, 7) ;
    wps = reshape( waypoint_data(idx, 8:end), 4, 5)' ;

    fig; clf;
    nexttile
    imshow(rgbimg) ; title('RGB') ;

    % nexttile
    % imshow(depthimg / 255) ; title('Depth raw (0~255 recaled)') ;

    nexttile
    imshow(depthimg_enhanced) ; title('Depth enhanced') ;

    nexttile
    plot(xy(1:idx,1), xy(1:idx,2), 'r.') ; hold on;

    if (is_joy_on)
        plot(px, py, 'oc', 'markersize', 12, 'MarkerFaceColor','r' ) ;
    else
        plot(px, py, 'oc', 'markersize', 12, 'MarkerFaceColor','c' ) ;
    end

    [hx, hy] = pol2cart(theta, 1) ;
    quiver( px0, py0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',32) ;
    quiver( px, py, hx, hy,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',24) ;
    quiver( px, py, hx, hy,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',24) ; 

    % sg
    plot( xy_sg(1)*12 + px, xy_sg(2)*12 + py, 'ms', 'MarkerSize',12, 'MarkerFaceColor', 'm') ;

    % wp
    if (is_joy_on)
        plot( wps(:,1)*12 + px, wps(:,2)*12 + py, 'r.', 'MarkerSize', 8 ) ;
    else
        plot( wps(:,1)*12 + px, wps(:,2)*12 + py, 'g.', 'MarkerSize', 8 ) ;
    end

    grid on; axis equal;  axis([px_min-11 px_max+11 py_min-11 py_max+11]); hold off;
    title('Odom pose and traj')

    % draw local
    nexttile

    if (is_joy_on)
        plot(0, 0, 'or', 'markersize', 10, 'MarkerEdgeColor','r', 'MarkerFaceColor', 'y', 'LineWidth', 2) ; hold on;
    else
        plot(0, 0, 'oc', 'markersize', 10, 'MarkerFaceColor','c' ) ; hold on;
    end

    [hx, hy] = pol2cart(0, 1) ;
    %quiver( 0, 0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',32) ;
    quiver( 0, 0, hy, hx/64,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ;

    plot( xy_sg(2), xy_sg(1), 'ms', 'MarkerSize',12, 'MarkerFaceColor','m') ;
    % wp
%    plot( wps(:,1)*40, wps(:,2)*40, 'g.', 'MarkerSize', 12 ) ;

    if (is_joy_on)
        plot( wps(:,2)*12 , wps(:,1)*12 , 'r.', 'MarkerSize', 8 ) ;
        legend('robot', '', 'SG ', 'colliding wpts (x20)', 'Location','SE')
    else
        plot( wps(:,2)*12 , wps(:,1)*12 , 'g.', 'MarkerSize', 8 ) ;
        legend('robot', '', 'SG ', 'wpts (x20)', 'Location','SE')
    end

    grid on; axis equal;  axis([-1 1 -1 1] * 0.75); hold off;
    title( sprintf('Subgoal (SG) and waypoints (wpts) \n w.r.t base-link'))


    drawnow ;
    set(gcf,'Position',[100 100 1024 1024])
    F(idx) = getframe(gcf) ;
    writeVideo(vidfile, F(idx)) ;
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




