
% read yaml

config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
config = ReadYaml(config_file) ;

% inpath = config.navdata_extractor.inpath ;
% base_sync_metadata_dir = config.navdata_extractor.outpath ;
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

sync_metadata_dir = sprintf('/media/results/navdata_collector/processed/%s/%s/synced',navtime_id, procssed_bag_ids{1}) ;

odom = load(sprintf("%s/sync_odom.txt", sync_metadata_dir)) ;
[num_data, c] = size(odom) ;

px_min = min(odom(:,5)) ;
px_max = max(odom(:,5)) ;
py_min = min(odom(:,6)) ;
py_max = max(odom(:,6)) ;

% odom: idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz 

fig = figure() ;
f.Position = [100 100 2048 2048];
t = tiledlayout("horizontal",'TileSpacing','Compact','Padding','Compact'); 

xy = zeros(num_data, 2) ;

% init pose
px0 = odom(1,5) ;
py0 = odom(1,6) ;
orient = quat2eul( odom(1, 8: 11) ) ; 
theta0 = orient(3)  ;
[hx0, hy0] = pol2cart(theta0, 1) ;

vidfile = VideoWriter('/home/hankm/Desktop/bag_sample') ;
vidfile.FrameRate = 40;
open(vidfile) ;

for idx=1:num_data-1

    fig; clf;
    rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
    depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;
    
    depthimg = rgb2gray(depthimg);
    % normlize depth img
    depthimg_enhanced = imadjust( double( depthimg ) / 255 ) ;

    % draw odom
    pose7 = odom(idx,5:11) ;
    orient = quat2eul( pose7(4:end) ) ;
    px = pose7(1);
    py = pose7(2); 
    theta = orient(3)  ;
    
    twist = odom(idx+1, 12:end) ;
    xy(idx,:) = [px, py] ;
    
    nexttile
    imshow(rgbimg) ; title('RGB') ;
    
    nexttile
    imshow(depthimg / 255) ; title('Depth raw (0~255 recaled)') ;

    nexttile
    imshow(depthimg_enhanced) ; title('Depth enhanced') ;
    
    nexttile
    plot(xy(1:idx,1), xy(1:idx,2), 'r.') ; hold on;
    plot(px, py, 'oc', 'markersize', 10, 'MarkerFaceColor','c' ) ;
    [hx, hy] = pol2cart(theta, 1) ;
    quiver( px0, py0, hx0*2, hy0*2,'AutoScale','off', 'Color', [0,1,0], 'LineWidth',2, 'MaxHeadSize',8) ;
    quiver( px, py, hx, hy,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ;
    quiver( px, py, hx, hy,'AutoScale','off', 'Color', [0,0,1], 'LineWidth',2, 'MaxHeadSize',8) ; 
    grid on; axis equal;  axis([px_min-11 px_max+11 py_min-1 py_max+1]); hold off;
    title('Odom pose and traj')

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
    writeVideo(vidfile, F(idx)) ;
    pause (0.005) 
end

close(vidfile) ;




