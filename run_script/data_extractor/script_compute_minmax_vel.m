
% read yaml

config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
config = ReadYaml(config_file) ;

inpath = config.navdata_extractor.inpath ;
base_sync_metadata_dir = config.navdata_extractor.outpath ;
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

% init pose
px0 = odom(1,5) ;
py0 = odom(1,6) ;
orient = quat2eul( odom(1, 8: 11) ) ; 
theta0 = orient(3)  ;
[hx0, hy0] = pol2cart(theta0, 1) ;

euc_dist = zeros(1,length(odom)) ;
ang_dist = zeros(1,length(odom)) ;
quat_dist = zeros(1, length(odom)) ;

pose7 = odom(idx,5:11) ;
orient = quat2eul( pose7(4:end) ) ;
px = pose7(1);
py = pose7(2); 
theta = orient(3)  ;
wHr_prev = xyzypr_to_htm( [px,py,0,orient] ) ;
vel_2d = zeros(num_data, 6) ;
dth = zeros(1, num_data) ;
quats = zeros(4, num_data) ;
for idx=1 : num_data-1
    idx
    rgbimg = imread( sprintf('%s/rgb%05d.png',sync_metadata_dir, idx-1) ) ;
    depthimg = imread(sprintf('%s/depth%05d.png',sync_metadata_dir, idx-1) )  ;
    
    % normlize depth img
    depthimg_enhanced = imadjust( double( depthimg ) / 65535 ) ;

    % draw odom
    pose7 = odom(idx,5:11) ;
    orient = quat2eul( pose7(4:end) ) ;
    px = pose7(1);
    py = pose7(2); 
    theta = orient(3)  ;

    vel_2d(idx,:) = odom(idx, [12:end] ) ; 

    twist = odom(idx+1, 12:end) ;
    xy(idx,:) = [px, py] ;

    wHr = xyzypr_to_htm([px, py, 0, orient]) ;

    if idx > 1
        r0Hr1 = inv(wHr_prev) * wHr ; 
        [dx, dy, dz, droll, dpit, dyaw] = htm_to_xyzypr( r0Hr1 ) ;
        euc_dist(idx) = sqrt(dx^2 + dy^2) ;
        ang_dist(idx) = dyaw ;

        q1 = htm_to_quat( wHr ) ; 
        q0 = htm_to_quat( wHr_prev) ;
        dth(idx) = acos(2*(q1' * q0 ) ^ 2 - 1 ) ;
        quats(:,idx) = q1' ;
%        max( norm(q, 2) , norm(q_,2) )  ;
    end

    wHr_prev = wHr ;
end

max_lindiff_per_frame = max( euc_dist ) ;  % 10 FPS (Hz)
min_lindiff_per_frame = min( euc_dist ) ;  % 10 FPS (Hz)

max_angdiff_per_frame = max( ang_dist * 180/ pi  ) ;
min_angdiff_per_frame = min( ang_dist * 180/ pi  ) ;

med_lindiff_per_frame = median( euc_dist ) ;  % 10 FPS (Hz)
med_angdiff_per_frame = median( ang_dist * 180/ pi  ) ;



