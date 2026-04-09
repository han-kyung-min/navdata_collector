% script to check the collision dataset

clear all; close all; clc;

wd = cd; %'/home/hankm/catkin_ws/src/navdata_collector/run_script/data_extractor/potmap2d';

% source folder
pkg_dir = sprintf('~/catkin_ws/src/navdata_collector'); %fileparts( fileparts( fileparts(pwd) ) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',pkg_dir);
config = ReadYaml(config_file) ;
base_dir = '/media/data/mydata/former_datasets/colldata/colldata-all/bag_2025-12-08-16-56-24' ;

data_dirs = dir( sprintf('%s/data*', base_dir) ) ;
core_file_list = {'context_index.txt',...
                  'corrected_waypoints_m.txt',...
                  'depth_sg.png',...
                  'new_subgoal_m.txt',...
                  'old_subgoal_m.txt',...        
                  'pose_context_m.txt',...       
                  'rgb_sg.png'} %,'traj_data.pkl'}

numdata = length(data_dirs) ;
for data_idx=1:numdata
    data_folder = sprintf('%s/%s',data_dirs(data_idx).folder, data_dirs(data_idx).name ) ;
    d = dir(data_folder);
    d = d(~[d.isdir]) ;       % keep files only (remove '.' '..' and folders)
    file_list = {d.name}.' ; 
    for ii=1:numel(core_file_list)
        p = fullfile(data_folder, core_file_list{ii}) ;
        assert( exist(p, 'file') )
    end
    fprintf('\r data%05d is checked', data_idx) ;
end

for data_idx=1 : numdata
    
    data_dir = sprintf('%s/%s',data_dirs(data_idx).folder, data_dirs(data_idx).name ) ;
    context_index = load( sprintf('%s/context_index.txt', data_dir) ) ;

    costmap_info_file = sprintf('%s/costmap_info.txt', data_dir) ;
    if isdir(costmap_info_file)
        fid = fopen(costmap_info_file) ;
        dummy = fgetl(fid) ; tline = fgetl(fid); fclose(fid) ;
        B = regexp(tline, '[-+]?(?:\d*\.\d+|\d+)', 'match');
        resolution = str2double(B{1}) ;
        map_size = str2num(B{2}) ;
        rx = map_size / 2;
        ry = rx ;
    else

    end

    costmap_file = sprintf('%s/costmap_i8.txt', data_dir) ;
    rgbimg_file  = sprintf('%s/rgb%05d.png', data_dir, context_index(end) ) ;
    pose_context_file = sprintf('%s/pose_context_m.txt', data_dir) ;
    corrected_waypoint_file = sprintf('%s/corrected_waypoints_m.txt', data_dir) ;
    pred_waypoint_file = sprintf('%s/pred_waypoints_m.txt', data_dir) ;
    old_sg_file = sprintf('%s/old_subgoal_m.txt', data_dir) ;
    new_sg_file = sprintf('%s/new_subgoal_m.txt', data_dir) ;

    costmap_i8 = load(costmap_file);
    rgb_img = imread( rgbimg_file  ) ;
    pose_context_m = load(pose_context_file) ;
    pred_waypts_m  = load(pred_waypoint_file) ;
    corr_waypts_m  = load(corrected_waypoint_file) ;
    old_sg_m = load(old_sg_file) ;
    new_sg_m = load(new_sg_file) ;

    pose_context_px = [pose_context_m(:,1:2) / resolution + [rx ry] pose_context_m(:,3:end) ] ;
    pred_waypts_px  = [pred_waypts_m(:,1:2) / resolution + [rx ry] pred_waypts_m(:,3:end) ] ;
    corr_waypts_px  = [corr_waypts_m(:,1:2) / resolution + [rx ry] corr_waypts_m(:,3:end) ] ;
    old_sgs_px      = [old_sg_m(:,1:2)/ resolution + [rx ry]  old_sg_m(:,3:end) ] ;
    new_sgs_px      = [new_sg_m(:,1:2)/ resolution + [rx ry]  new_sg_m(:,3:end) ] ;

    % draw
    fig = figure(1); clf;
    fig.Position = [2800, 400, 1400, 640]
    t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(t, 1);
    imshow(rgb_img) ;

    X = corr_waypts_px(:,1);
    Y = corr_waypts_px(:,2);
    % direction from each point to the next
    d   = [diff(X), diff(Y)];          % N-1 directions
    d   = [d; 0 0];                     % pad last one
    nrm = max(hypot(d(:,1), d(:,2)), eps);
    dir = d ./ nrm;                     % unit directions
    L = 10;                              % arrow length (in your data units, e.g. pixels)
    U = L * dir(:,1);
    V = L * dir(:,2);

    nexttile(t,2) ;
    imagesc(costmap_i8) ; colormap('gray'); colorbar ;
    hold on
    plot(rx, ry, 'go', 'markerfacecolor', 'g', 'markersize', 8) ;
    plot(corr_waypts_px(:,1) , corr_waypts_px(:,2) , 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8 ); hold on
    quiver(X(2), Y(2), U(2), V(2), 0, 'c', 'LineWidth',1, 'MaxHeadSize',2);

    plot(pred_waypts_px(:,1) , pred_waypts_px(:,2) , 'ys', 'MarkerFaceColor', 'y', 'MarkerSize', 4 );
    plot(old_sgs_px(:,1), old_sgs_px(:,2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm' ) ;
    plot(new_sgs_px(:,1), new_sgs_px(:,2), 'bs', 'MarkerSize', 10 , 'MarkerFaceColor', 'b') ;

    lgd = legend('Robot', 'New Waypoints', 'Old Waypoints', 'Old SG', 'New SG', 'Location', 'NW') ;
    lgd.Color = [0.8 0.8 0.8];
    pause

    %outname = sprintf('%s/outres%04d.png', out_folder, data_idx) ;
    %exportgraphics(gcf, outname, 'ContentType', 'vector', 'BackgroundColor','none');

end