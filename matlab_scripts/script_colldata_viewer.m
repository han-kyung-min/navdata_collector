
clear all; close all; clc;

% hyper params

draw_topomap = true ;
context_len = 5;
eta = 1.0 ;             % Repulsive potential scaling factor (η)
rho0 = 1.2 / 0.05 ;     % Influence distance (ρ0, # grids )
obs_thr = 99 ;
FPS = 10 ;
v_max = 0.3 ; % 0.3 m/s
w_max = 0.6 ; % 0.3 rad/s
ws = 3 ;

resolution = 0.05 ;
map_size_m = 10.0 ;
robot_radius_m = 0.25 ;
inflation_radius_m = 1.0 ;
max_range_m = 25;

cm_params = struct('resolution', resolution, 'map_size_m', map_size_m, 'robot_radius_m', robot_radius_m, ...
    'inflation_radius_m', inflation_radius_m, 'max_range_m', max_range_m ) ;

map_size_px = map_size_m / resolution ; 
rx = map_size_px / 2 ;  % robot position (center of the map)
ry = rx ;

% source folder
pkg_dir = fileparts( fileparts( fileparts(pwd) ) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',pkg_dir);
config = ReadYaml(config_file) ;
colldata_dir = '/media/data/mydata/former_datasets/colldata/colldata-all' ;
bag_dirs = dir(sprintf('%s/bag_*', colldata_dir)) ;

for bag_idx = 82:length(bag_dirs)
    bag_dir = bag_dirs(bag_idx) ;

    data_path = sprintf('%s/%s', colldata_dir, bag_dir.name) ; 
    colldatalist = dir(sprintf('%s/data*', data_path)) ; 
    topomap_name_tmp = dir(sprintf('%s/T*', data_path)) ;
    topomap_name = topomap_name_tmp.name ;
    
    proc_dir = sprintf('/media/data/results/navdata_collector/colldata/processed/%s_coll', topomap_name) ;
    
    str_split = split(bag_dir.name, '_') ;
    navtime_yymmddhhss = str_split{end} ;
    navtime_tmp = split(navtime_yymmddhhss,'-') ;
    navtime_yymmddhh = join( string(navtime_tmp(1:end-1)), '-' ) ;
    
    %sync_metadata_dir = sprintf('%s/coll_%s/bag_%s/synced', proc_dir, navtime_yymmddhh, navtime_yymmddhhss) ; % r9-2 (R)
    topomap_root_dir = sprintf('/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/%s',topomap_name) ;
     
    
    rgb_folder = sprintf('%s/rgb*.png', topomap_root_dir)  ;
    D      = dir(rgb_folder);
    names  = {D.name} ;                 % cell array of ALL file names
    S = lower(string(names));           % string array for endsWith
    
    for idx=1:length(S)
        s= S{idx} ;
        digits_str = regexprep(s, '\D', '') ;   % -> '0000'
        topoimg_idxstr{idx} = digits_str ;
    end
    
    % load topomap
    topo_odom_file = sprintf('%s/topo_odom.txt', data_path) ;
    topo_m2b_file = sprintf('%s/topo_tf_m2b.txt', data_path) ;
    topo_m2o_file = sprintf('%s/topo_tf_m2o.txt', data_path) ;
    [topo_odom_raw, topo_odom_xy, topo_o1Hb ] = load_pose_data(topo_odom_file) ; 
    [topo_m2b_raw, topo_m2b_xy, topo_m1Hb ] = load_pose_data(topo_m2b_file) ; 
    [topo_m2o_raw, topo_m2o_xy, topo_m1Ho ] = load_pose_data(topo_m2o_file) ; 
    
    [num_node, ~] = size(topo_m2b_raw ) ;
    nav_m2b_file = sprintf('%s/sync_tf_m2b.txt', data_path) ;
    nav_m2o_file = sprintf('%s/sync_tf_m2o.txt', data_path) ;
    nav_odom_file = sprintf('%s/sync_odom.txt', data_path) ;
    
    % copy nav pose files to colldata dir
    [nav_odom_raw, nav_odom_xy, nav_o2Hb ] = load_pose_data(nav_odom_file) ; 
    [nav_m2b_raw,  nav_m2b_xy,  nav_m2Hb ] = load_pose_data(nav_m2b_file) ; 
    [nav_m2o_raw,  nav_m2o_xy,  nav_m2Ho ] = load_pose_data(nav_m2o_file) ; 
    [nav_len, c] = size(nav_odom_raw) ;


    num_coll_data = length(colldatalist) ; 
    data_idx = 1 ;
    while data_idx <= num_coll_data %num_data-1
    
        %% load coll data
        colldata_name = colldatalist(data_idx).name ;
        colldata_path = sprintf('%s/%s', data_path, colldata_name) ;
    
        % 1. context idx
        context_index = load( sprintf('%s/context_index.txt', colldata_path) ) ;
    
        % 2. label waypoints
        corrected_waypoints_m = load( sprintf('%s/corrected_waypoints_m.txt',colldata_path) ) ;
    
        % 3. pred waypoints
        pred_waypoints_m = load( sprintf('%s/pred_waypoints_m.txt',colldata_path) ) ;
    
        % 4. data sg indexs 
        tmp = load( sprintf('%s/data_sg_idx.txt', colldata_path) ) ;
        img_data_idx_1 = tmp(1) ; new_sg_idx_1 = tmp(2) ; global_sg_idx = tmp(3)  ;
        img_idx = img_data_idx_1 ;
        sg_idx = new_sg_idx_1 + 1;
        % 5. new sg pose
        new_subgoal_m = load( sprintf('%s/new_subgoal_m.txt', colldata_path) ) ; % x, y, qw, qz
    
        % 6. old sg pose
        old_subgoal_m = load( sprintf('%s/old_subgoal_m.txt', colldata_path) ) ; % x, y, qw, qz
    
        % 7. pose_context
        pose_context_m = load( sprintf('%s/pose_context_m.txt', colldata_path) ) ; % x, y, qw, qz
    
        % 8. costmap i8
        costmap_i8 = load(sprintf('%s/costmap_i8.txt', colldata_path)) ;
        
        % 9. topomap
        if draw_topomap
    
            o1Hb_curr  = topo_o1Hb(:,:,sg_idx) ; 
            m1Ho1_curr = topo_m1Ho(:,:,sg_idx) ; % odom wrt map1 @ curr sgidx
            m1Hsg_curr = topo_m1Hb(:,:,sg_idx) ; % sg pose wrt map1 @ curr sgidx
            
            o2Hb_curr  = nav_o2Hb(:,:,img_idx) ;
            m2Ho2_curr = nav_m2Ho(:,:,img_idx) ;
            m2Hb_curr  = nav_m2Hb(:,:,img_idx) ;
        
            % draw future sgs
        
            sgs_xyzq_corrected_px = zeros(num_node, 7) ; % x y z qw qx qy qz
            for next_sg_idx=1:num_node
                bHsg = inv( m2Hb_curr ) * topo_m1Hb(:,:,next_sg_idx) ; 
                sgs_xyzq_corrected_px(next_sg_idx,1:2) = bHsg(1:2,4)' / resolution ;
                sgs_xyzq_corrected_px(next_sg_idx,4:end) = htm_to_quat( bHsg ) ;
            end
            [num_sgs, ~] = size(sgs_xyzq_corrected_px) ;
            assert (num_sgs > 0) ;
        end
    
        % load image
        depth_img = double( imread(sprintf('%s/depth%05d.png', colldata_path, img_idx-1)) ) / 1000 ;
        rgb_img = imread(sprintf('%s/rgb%05d.png', colldata_path, img_idx-1 )) ;
    
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% ===== drawing data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        waypt_scale = 4;
        sg_idx = new_sg_idx_1 + 1 ; 
    
        %% === Step 5: plotting  === %%  
        arrow_step = 1; arrow_scale = 20;
    
        fig = figure(1); clf;
        fig.Position = [2800, 300, 1600, 780] ;
        tmain = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        title(tmain, sprintf('Bag ID: %s', navtime_yymmddhh),'FontWeight','bold','FontSize',14, 'Interpreter','none');
        
        % left column: 2x2 img grid
        leftgrid = tiledlayout(tmain, 2,2, 'TileSpacing', 'compact', 'Padding', 'compact');
        leftgrid.Layout.Tile = 1;
    
        nexttile(leftgrid, 1);
        imshow(rgb_img) ; % obs rgb 
        title('Observed RGB [{\color{green}▶ }]') ;
    
        nexttile(leftgrid,2);  % subgoal
        sgimg_idx = topoimg_idxstr{sg_idx} ;
        rgb_sg_file =  sprintf('%s/rgb%s.png',topomap_root_dir, sgimg_idx) ;
        depth_sg_file =  sprintf('%s/depth%s.png',topomap_root_dir, sgimg_idx) ;
        rgb_sg = imread(rgb_sg_file) ;
        depth_sg = double( imread(depth_sg_file) ) / 1000 ;
        imshow(rgb_sg)  ;
        title('SG (corrected) RGB [{\color{red}●}]')
    
        nexttile(leftgrid,3);  % obs depth
        draw_depth_colormap( depth_img, 0.05, 3.0, 0.5 ) ;
    
        ax4=nexttile(leftgrid,4);  % sg depth
        draw_depth_colormap( depth_sg, 0.05, 3.0, 0.5 ) ;
    
        nexttile(tmain, 2);
        imshow( int8(costmap_i8) );  % Show potential field as a heatmap
        hold on;
        
        % 0. Robot
        p0 = plot( rx, ry, 'g>', 'MarkerSize', 15, 'MarkerFaceColor', 'g' ) ;
    
        % 1. Topomap
        if draw_topomap
            p1 = drawSubGoalPoses(sgs_xyzq_corrected_px(1:2:end,:), rx, ry, 2) ; % draw SLAM SGs
        end
    
        % 2. draw label waypoints 
        corrected_waypoints_px = corrected_waypoints_m ;
        corrected_waypoints_px(:,1:2) = corrected_waypoints_px(:,1:2) / resolution  ;
        p2 = plot( rx + corrected_waypoints_px(:,1) * waypt_scale , ry + corrected_waypoints_px(:,2) * waypt_scale, 'ys', 'MarkerFaceColor','y') ;
    
        % 3. draw predict waypoints
        pred_waypoints_px = [pred_waypoints_m(:,1:2)/resolution zeros(5, 1) pred_waypoints_m(:,3), zeros(5,2), pred_waypoints_m(:,end)] ;
        pred_waypoints_px(:,1:2) = pred_waypoints_px(:,1:2)* waypt_scale ;
        pred_waypoints_px_shift = [pred_waypoints_px(:,1:2)+rx, pred_waypoints_px(:,3:end)] ;
        p3 = plot(pred_waypoints_px_shift(:,1), pred_waypoints_px_shift(:,2), 'ms', 'MarkerFaceColor','m') ; 
    
        % 4. draw old sg
        old_subgoal_px = old_subgoal_m ;
        old_subgoal_px(:,1:2) = old_subgoal_px(:,1:2)/ resolution ; 
        p4 = plot( rx + old_subgoal_px(1), ry + old_subgoal_px(2),  'mo', 'MarkerSize', 12 , 'MarkerFaceColor', 'm') ;
    
        % 5. draw label(new) sg
        new_subgoal_px = new_subgoal_m ;
        new_subgoal_px(:,1:2) = new_subgoal_px(:,1:2)/ resolution ;
        p5 = plot( rx + new_subgoal_px(1), ry + new_subgoal_px(2),  'co', 'MarkerSize', 12 , 'MarkerFaceColor', 'c') ;
    
        colorbar; 
        title( sprintf('Data idx: %d / %d  ',data_idx, num_coll_data)) ;
        hold on;
    
        if draw_topomap
            lgd = legend([p0,p1,p2,p3,p4,p5 ], ...
                'Robot', 'Topomap(SLAM)','Label Waypts', 'Pred Waypts', 'SG(pred)', 'SG(corr)', 'Location', 'NW')
            lgd.Color = [0.8 0.8 0.8];
            set(gca,'YDir','normal');
        else
            lgd = legend([p0,p2,p3,p4,p5 ], ...
                'Robot','Label Waypts', 'Pred Waypts', 'SG(pred)', 'SG(corr)', 'Location', 'NW')
            lgd.Color = [0.8 0.8 0.8];
            set(gca,'YDir','normal');
        end
       
        % === filter subgoals based on its orientation if necessary
        % sg_vidx = 1:num_sgs ;
        % if angle_filter == true
        %     sgs_quat = sgs_xyzq_corrected_px(:,4:end) ;
        %     qdist = zeros(num_sgs,1) ;
        %     for ii =1:num_sgs
        %         qdist(ii) = quat_distance(sgs_quat(ii,:), [1,0,0,0]) ;
        %     end
        %     sg_vidx = find( qdist < pi/2 );
        %     %sgs_xyzq_corrected_px = sgs_xyzq_corrected_px(vidx, : ) ;
        % end
    
        outfig_dir = sprintf('/media/data/results/devgru/colldata_viewer/%s',navtime_yymmddhh) ;
        mkdir(outfig_dir) ;
        outfig_name = sprintf('%s/data%05d.png',outfig_dir, data_idx) ;
        exportgraphics(gcf, outfig_name, 'Resolution', 300, 'BackgroundColor', 'none', 'ContentType', 'image');
    
        data_idx = data_idx + 1;
        
    end
end