% script  trajectory data collection

clear all; close all; clc;

% source folder
pkg_dir = '~/catkin_ws/src/navdata_collector'%fileparts( fileparts( fileparts(pwd) ) ) ;
proc_dir = '/media/results/navdata_collector/colldata/processed' ;

topomap_name = 'T10' ;  %  may be redo T9 4,5,  T1 is problem 

bag_group_dir = sprintf('%s/%s_coll', proc_dir, topomap_name) ;
bag_dirs = dir( sprintf('%s/coll_*/bag_*', bag_group_dir) ) ;
bag_dir = bag_dirs(end) ;
sync_metadata_dir = sprintf('%s/%s/synced', bag_dir.folder, bag_dir.name) ; % r9-2 (R)

col_base_dir = sprintf('/media/data/mydata/former_datasets/colldata/colldata-all');
col_data_dirs = dir(sprintf('%s/bag_*',col_base_dir) ) ;
col_data_dir = col_data_dirs(end) ;

topomap_root_dir = sprintf('~/python_ws/viznav/depth-nav/deployment/topomaps/%s',topomap_name) ;
rgb_folder = sprintf('%s/rgb*.png', topomap_root_dir)  ;
D      = dir(rgb_folder);
names  = {D.name} ;                 % cell array of ALL file names
S = lower(string(names));                   % string array for endsWith

for idx=1:length(S)
    s= S{idx} ;
    digits_str = regexprep(s, '\D', '') ;   % -> '0000'
    topoimg_idxstr{idx} = digits_str ;
end

% load topomap
topo_odom_file = sprintf('%s/topo_odom.txt', topomap_root_dir) ;
topo_m2b_file = sprintf('%s/topo_tf_m2b.txt', topomap_root_dir) ;
topo_m2o_file = sprintf('%s/topo_tf_m2o.txt', topomap_root_dir) ;
[topo_odom_raw, topo_odom_xy, topo_o1Hb ] = load_pose_data(topo_odom_file) ; 
[topo_m2b_raw, topo_m2b_xy, topo_m1Hb ] = load_pose_data(topo_m2b_file) ; 
[topo_m2o_raw, topo_m2o_xy, topo_m1Ho ] = load_pose_data(topo_m2o_file) ; 

[num_node, ~] = size(topo_m2b_raw ) ;

data_folder = sprintf('%s/%s', col_data_dir.folder,  col_data_dir.name) ; 
num_data = length(dir(sprintf('%s/data*', data_folder))) ;

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

%o1Ho2 = topo_o1Hb(:,:,1) * inv(o2Hb(:,:,1))  ;

%% === Step 2: Compute ρ(q) = distance to the nearest obstacle ===
% bwdist() returns the Euclidean distance in pixels from each free cell
% to the nearest obstacle cell.

map_size_px = map_size_m / resolution ; 
rx = map_size_px / 2 ;  % robot position (center of the map)
ry = rx ;

% load subgoal
% idx, seq, s, ns, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
data_cnt = 0;

for data_idx = 1 :num_data

    %% === 1. Read metadata == %%
    data_folder = sprintf('%s/%s/data%05d',col_data_dir.folder, col_data_dir.name, data_idx-1) ;
    sg_idx_ = load( sprintf('%s/data_sg_idx.txt',data_folder) ) ;  % +1 matlab conv
    sg_idx = sg_idx_(2) ;
    old_sg_m = load( sprintf('%s/old_subgoal_m.txt', data_folder) ) ;
    old_sg_px = old_sg_m(1:2) /resolution ; % old sg_px

    context_index = load(sprintf('%s/context_index.txt', data_folder) ) ;
    img_idx = context_index(end) ;

    % We go ahead do the data collection if the joy msg is on
    depth_img = double( imread(sprintf('%s/depth%05d.png', data_folder, img_idx)) ) / 1000 ;
    rgb_img = imread(sprintf('%s/rgb%05d.png', data_folder, img_idx )) ;

    pred_waypt_m = load(sprintf('%s/pred_waypoints_m.txt', data_folder)) ;
    pred_waypt_px = pred_waypt_m(:,1:2) / resolution ;
    pred_waypt_px = [pred_waypt_px pred_waypt_m(:,3) zeros(5, 3) pred_waypt_m(:,4) ] ;

    corr_waypt_m = load(sprintf('%s/corrected_waypoints_m.txt', data_folder)) ; 
    corr_waypt_px = corr_waypt_m(:,1:2) / resolution ;
    corr_waypt_px = [corr_waypt_px corr_waypt_m(:,3) zeros(5, 3) corr_waypt_m(:,4) ] ;
    tgt_wx_px = corr_waypt_px(2,1) ; % 2nd wpt
    tgt_wy_px = corr_waypt_px(2,2)  ; % 
    attr_ang_rad = atan2(tgt_wy_px, tgt_wx_px) ;

    scandata_file = sprintf('%s/scan%05d.txt', sync_metadata_dir, img_idx) ;
    scandata = load( scandata_file ) ;
    ranges = scandata(:,2) ;
    angles = scandata(:,1) ;

    %% === 2. Build costmap === %%
    [costmap_i8, costmap_u8] = build_costmap( ranges, angles, cm_params) ;
    ukn_idx = find(costmap_i8 == -1) ;
    obstacle_mask = costmap_i8 >= obs_thr;
    rho = bwdist(obstacle_mask) ; %resolution;  % ρ(q) in meters
    
    %% === 3. Compute Repulsive Potential U_rel(q) ===
    % U_rep(q) = 0.5 * η * (1/ρ(q) - 1/ρ0)^2 if ρ(q) <= ρ0, else 0.
    Urep = zeros(size(rho));
    mask = (rho <= rho0) & (rho > 0);  % Valid region of influence
    Urep(mask) = 0.5 * eta * (1 ./ rho(mask) - 1/rho0).^2;
    
    %% === 4: Compute the Rep Potential Fields (force direction) ===
    % ∇ρ(q) points outward from obstacles. Normalize it to get direction.
    [dRhoX, dRhoY] = gradient(rho, 1);  % Derivatives of ρ(q)
    grad_norm = sqrt(dRhoX.^2 + dRhoY.^2) + 1e-8;
    dir_x = dRhoX ./ grad_norm;   % Unit vector in x-direction
    dir_y = dRhoY ./ grad_norm;   % Unit vector in y-direction
    
    % F_rel(q) = η * (1/ρ(q) - 1/ρ0) * (1/ρ(q)^2) * ∇ρ(q)
    Frx = zeros(size(rho));
    Fry = zeros(size(rho));
    Fr_magnitude = eta * (1 ./ rho(mask) - 1/rho0) ./ (rho(mask).^2);
    Frx(mask) = Fr_magnitude .* dir_x(mask);
    Fry(mask) = Fr_magnitude .* dir_y(mask);   
    angle_map = rad2deg(atan2(Fry, Frx)) ;

    [costmap_roi, angle_map_roi, roi_front_px, roi_back_px] = set_roi(costmap_i8,  angle_map, cm_params) ;

    % roi flow
    val_roi_idx = find(costmap_roi >= 0) ;
    roi_repflow_deg = median( angle_map_roi(val_roi_idx) ) ;
    roi_repflow_rad = roi_repflow_deg * pi / 180 ;
    attr_flow_deg = attr_ang_rad * 180 / pi ;
    roi_repflow_u = cos(roi_repflow_rad ) ;
    roi_repflow_v = sin(roi_repflow_rad ) ;

    %% === Step 5: plotting  === %%  
    arrow_step = 1; arrow_scale = 20;

    fig = figure(1); clf;
    fig.Position = [2800, 300, 1600, 780] ;
    tmain = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tmain, sprintf('Bag ID: %s', bag_dir.name),'FontWeight','bold','FontSize',14, 'Interpreter','none');
    
    % left column: 2x2 img grid
    leftgrid = tiledlayout(tmain, 2,2, 'TileSpacing', 'compact', 'Padding', 'compact');
    leftgrid.Layout.Tile = 1;

    nexttile(leftgrid, 1);
    imshow(rgb_img) ; % obs rgb 
    title('Observed RGB [{\color{green}▶ }]') ;

    nexttile(leftgrid,2);  % subgoal
    %sgimg_idx = topoimg_idxstr{sg_idx} ;
    rgb_sg_file =  sprintf('%s/rgb_sg.png',data_folder) ;
    depth_sg_file =  sprintf('%s/depth_sg.png',data_folder) ;
    rgb_sg = imread(rgb_sg_file) ;
    depth_sg = double( imread(depth_sg_file) ) / 1000 ;
    imshow(rgb_sg)  ;
    title('SG (corrected) RGB [{\color{red}●}]')

    nexttile(leftgrid,3);  % obs depth
    draw_depth_colormap( depth_img, 0.05, 3.0, 0.5 ) ;

    ax4=nexttile(leftgrid,4);  % sg depth
    draw_depth_colormap( depth_sg, 0.05, 3.0, 0.5 ) ;

    nexttile(tmain, 2);
    imshow( costmap_i8 );  % Show potential field as a heatmap
    hold on;
    
    %drawSubGoalPoses(sgs_xyzq_corrected_px(sg_vidx,:), rx, ry, 2) ; % draw SLAM SGs

    plot( rx, ry, 'g>', 'MarkerSize', 15, 'MarkerFaceColor', 'g' ) ;
    plot( rx + pred_waypt_px(:,1) * 2 , ry + pred_waypt_px(:,2) * 2, 'ys', 'MarkerFaceColor','y') ;
%    plot( sgs_px_cands(:,1), sgs_px_cands(:,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b' ) ;
    plot( rx + old_sg_px(1), ry + old_sg_px(2),  'mo', 'MarkerSize', 8 , 'MarkerFaceColor', 'm') ;

    colorbar; 
    title( sprintf('Data idx: %d / %d ,  # Collected Data: %d ',data_idx, num_data, data_cnt)) ;
    hold on;

    % draw GT topomap 
    % new_sg_idx = sg_idx; %find(dist_to_sg_cands == min(dist_to_sg_cands)) ;
    % new_sg_xyzq_px = sgs_xyzq_corrected_px(new_sg_idx, :) ;
    % new_sg_xyzq_m  = new_sg_xyzq_px ;
    % new_sg_xyzq_m(1:3)  = new_sg_xyzq_m(1:3) * resolution ;
    % plot( new_sg_xyzq_px(1) + rx, new_sg_xyzq_px(2) + ry, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'm' ) ;

    %lgd = legend('Pred Waypts', 'SG(SLAM)', 'Robot', 'SG(pred)', 'SG(corr)', 'Location', 'NW')
    set(gca,'YDir','normal');

    % Draw new SG == %%
    new_sg_m = load(sprintf('%s/new_subgoal_m.txt', data_folder)) ;

    gx = new_sg_m(1) / resolution + rx ;
    gy = new_sg_m(2) / resolution + rx ;
    
    plot( gx, gy,  'cs', 'MarkerSize', 12 , 'LineWidth', 3) ;
    set(gcf, 'pointer', 'crosshair'); 
    set(gcf, 'pointer', 'arrow');

    waypt_traj_px_tform = corr_waypt_px ;
    waypt_traj_px_tform(:,1:2) = waypt_traj_px_tform(:,1:2)*2  ;
    %drawPoses2D(waypt_traj_px_tform) ; 
    plot( rx + waypt_traj_px_tform(:,1) * 2 , ry + waypt_traj_px_tform(:,2) * 2, 'gs', 'MarkerFaceColor','g') ;

    lgd = legend('Robot', 'Pred Waypts', 'SG(pred)', 'SG(corr)', 'Location', 'NW')
    lgd.Color = [0.8 0.8 0.8];

    pause;

    data_cnt = data_cnt + 1 ;

end