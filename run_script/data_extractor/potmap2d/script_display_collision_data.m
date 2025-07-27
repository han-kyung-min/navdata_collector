% script to check the collision dataset

clear all; close all; clc;

% source folder
pkg_dir = fileparts( fileparts( fileparts(pwd) ) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',pkg_dir);
config = ReadYaml(config_file) ;
base_dir = '/media/data/mydata/former_datasets/colldata' ;

bag_dirs = dir( sprintf('%s/bag_*', base_dir) ) ;
bag_dir = sprintf('%s/%s', bag_dirs.folder, bag_dirs.name ) ;

colldata_dirs = dir( sprintf('%s/data*', bag_dir)) ;

for data_idx=1 : length(colldata_dirs)
    
    colldata_dir = sprintf('%s/%s',colldata_dirs(data_idx).folder, colldata_dirs(data_idx).name ) ;
    context_index = load( sprintf('%s/context_index.txt', colldata_dir) ) ;

    costmap_info_file = sprintf('%s/costmap_info.txt', colldata_dir) ;
    fid = fopen(costmap_info_file) ;
    dummy = fgetl(fid) ; tline = fgetl(fid); fclose(fid) ;
    B = regexp(tline, '[-+]?(?:\d*\.\d+|\d+)', 'match');
    resolution = str2double(B{1}) ;
    map_size = str2num(B{2}) ;
    rx = map_size / 2;
    ry = rx ;

    costmap_file = sprintf('%s/costmap_i8.txt', colldata_dir) ;
    rgbimg_file  = sprintf('%s/rgb%05d.png', colldata_dir, context_index(end) ) ;
    pose_context_file = sprintf('%s/pose_context.txt', colldata_dir) ;
    corrected_waypoint_file = sprintf('%s/corrected_waypoints.txt', colldata_dir) ;
    pred_waypoint_file = sprintf('%s/pred_waypoints.txt', colldata_dir) ;

    costmap_i8 = load(costmap_file);
    rgb_img = imread( rgbimg_file  ) ;
    pose_context = load( pose_context_file )  ;
    pred_waypts = load(pred_waypoint_file) ;
    corr_waypts = load(corrected_waypoint_file) ;

    % draw
    fig = figure(4); clf;
    fig.Position = [2800, 400, 1400, 640]
    t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(t, 1);
    imshow(rgb_img) ;

    nexttile(t,2) ;
    imagesc(costmap_i8) ; colormap('gray'); colorbar ;
    hold on
    plot(rx, ry, 'co', 'markerfacecolor', 'c', 'markersize', 8) ;
    plot(corr_waypts(:,1) / resolution, corr_waypts(:,2) / resolution, 'gs', 'MarkerFaceColor', 'g',  'MarkerSize', 4 ) ;
    plot(pred_waypts(:,1) / resolution, pred_waypts(:,2) / resolution, 'ys', 'MarkerFaceColor', 'y', 'MarkerSize', 4 );

    pause(.1)

end