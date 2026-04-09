% script  trajectory data collection

clear all; close all; clc;
save_data = true ;

% source folder
pkg_dir = fileparts( fileparts( fileparts(pwd) ) ) ;
proc_dir = '/media/results/navdata_collector/colldata/processed' ;
addpath('/home/hankm/matlab_ws/NavdataCollector') ;
% config_file = sprintf('%s/param/navdata_collector.yaml',pkg_dir);
% config = ReadYaml(config_file) ;

topomap_name = 'T10' ;  %  may be redo T9 4,5,  T1 is problem 

if strcmp(topomap_name, 'T1')
    angle_filter = true ;  
else
    angle_filter = false;
end

bag_group_dir = sprintf('%s/%s_coll', proc_dir, topomap_name) ;
bag_dirs = dir( sprintf('%s/coll_*/bag_*', bag_group_dir) ) ;
out_base_dir = sprintf('/media/data/mydata/former_datasets/colldata/colldata-all');
% write readme.txt file

if save_data == true
    fid = fopen( sprintf('%s/readme.txt', out_base_dir), 'w') ;
    fprintf(fid, 'Collision data info. They are all defined w.r.t current robot pose\n') ;
    fprintf(fid, 'corrected_waypoints_m.txt: \t  x, y, qw, qz \n') ;
    fprintf(fid, 'pred_waypoints_m.txt:      \t  x, y, qw, qz \n') ;
    fprintf(fid, 'pose_context_m.txt:        \t  x, y, 0, qw, qx, qy, qz\n') ;
    fprintf(fid, 'subgoals_m.txt:            \t  x_old, y_old, 0, 0\n x_corr, y_corr, qw, qz\n ') ;
    fclose(fid) ;
end

bag_dir = bag_dirs(end) ; 
sync_metadata_dir = sprintf('%s/%s/synced', bag_dir.folder, bag_dir.name) ; % r9-2 (R)
topomap_root_dir = sprintf('/home/hankm/python_ws/viznav/devgru/deployment/topomaps/%s',topomap_name) ;

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

nav_m2b_file = sprintf('%s/sync_tf_m2b.txt', sync_metadata_dir) ;
nav_m2o_file = sprintf('%s/sync_tf_m2o.txt', sync_metadata_dir) ;
nav_odom_file = sprintf('%s/sync_odom.txt', sync_metadata_dir) ;

% copy nav pose files to colldata dir

[nav_odom_raw, nav_odom_xy, nav_o2Hb ] = load_pose_data(nav_odom_file) ; 
[nav_m2b_raw,  nav_m2b_xy,  nav_m2Hb ] = load_pose_data(nav_m2b_file) ; 
[nav_m2o_raw,  nav_m2o_xy,  nav_m2Ho ] = load_pose_data(nav_m2o_file) ; 

[num_data, c] = size(nav_odom_raw) ;

str_split = split(sync_metadata_dir, '/') ;
navtime_id = str_split{end-1} ;
colldata_dir = sprintf('%s/%s', out_base_dir, navtime_id) ; 

if save_data == true
    if isdir(colldata_dir) 
        %rmdir(colldata_dir, 's') ;
        error('%s  dir exist!! \n Make sure to remove this dir before processing the data \n', colldata_dir) ;
    else
        mkdir(colldata_dir) ;
    end

    topomap_name_file = sprintf('%s/%s',colldata_dir, topomap_name) ;
    system(sprintf('touch %s', topomap_name_file)) ;
    
    % copy topo files
    cmd = sprintf('cp %s/topo_*.txt %s/',topomap_root_dir, colldata_dir) ;
    system(cmd) ;
    
    % copy nav files
    cmd = sprintf('cp %s/sync_*.txt %s/',sync_metadata_dir, colldata_dir) ;
    system(cmd) ;
end


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

% load waypoints
navdata = load(sprintf('%s/sync_navdata.txt', sync_metadata_dir)) ;

%% === Step 2: Compute ρ(q) = distance to the nearest obstacle ===
% bwdist() returns the Euclidean distance in pixels from each free cell
% to the nearest obstacle cell.

map_size_px = map_size_m / resolution ; 
rx = map_size_px / 2 ;  % robot position (center of the map)
ry = rx ;

% load subgoal
% idx, seq, s, ns, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
subgoals = load(sprintf('%s/sync_rel_subgoals.txt', sync_metadata_dir)) ;
data_cnt = 0;

coll_attention_idxs_init = find( navdata(:, 5) ) ; %  1 ~ num_rgb

% add one pads @ the beginning of "joy_on"  
v = navdata(:, 5)' ;
joy_start_idx = find(diff([0 v] == 1) ) ;
k = 16; % 1.6 sec ahead of collsion. (10 FPS)
J = joy_start_idx(:) + (-k:-1);          % size: [num_starts x k]
J = J(J >= 1 & J <= numel(v));       % clip to bounds
v2 = v;
v2(J) = 1;

coll_attention_idxs = find(v2) ;
step_idx = 1;

while step_idx <= length(coll_attention_idxs) %num_data-1
    data_idx = coll_attention_idxs(step_idx) ;
    %% === 1. Read metadata == %%
    navdata_line = navdata(data_idx,:) ;
    %joy_on = waypt_line(5) ;
    sg_idx = navdata_line(6) + 1 ;  % +1 matlab conv
    old_sg_px = [subgoals(data_idx,5:6) /resolution subgoals(data_idx,[11, 10]) ] ; % old sg_px

    o1Hb_curr  = topo_o1Hb(:,:,sg_idx) ; 
    m1Ho1_curr = topo_m1Ho(:,:,sg_idx) ; % odom wrt map1 @ curr sgidx
    m1Hsg_curr = topo_m1Hb(:,:,sg_idx) ; % sg pose wrt map1 @ curr sgidx
    
    o2Hb_curr  = nav_o2Hb(:,:,data_idx) ;
    m2Ho2_curr = nav_m2Ho(:,:,data_idx) ;
    m2Hb_curr  = nav_m2Hb(:,:,data_idx) ;

    %bHsg = inv( m2Hb_curr ) * m1Hsg_curr ;
    %sgs_corrected_px = bHsg(1:2,4)' / resolution ;
    
    % draw future sgs

    sgs_xyzq_corrected_px = zeros(num_node, 7) ; % x y z qw qx qy qz
    for next_sg_idx=1:num_node
        bHsg = inv( m2Hb_curr ) * topo_m1Hb(:,:,next_sg_idx) ; 
        sgs_xyzq_corrected_px(next_sg_idx,1:2) = bHsg(1:2,4)' / resolution ;
        sgs_xyzq_corrected_px(next_sg_idx,4:end) = htm_to_quat( bHsg ) ;
    end
    [num_sgs, ~] = size(sgs_xyzq_corrected_px) ;
    assert (num_sgs > 0) ;

    % if ~joy_on % skip if joy button (deadman switch) is not pressed
    %     continue ;
    % end

    img_idx = data_idx - 1 ;

    % We go ahead do the data collection if the joy msg is on
    depth_img = double( imread(sprintf('%s/depth%05d.png', sync_metadata_dir, img_idx)) ) / 1000 ;
    rgb_img = imread(sprintf('%s/rgb%05d.png', sync_metadata_dir, img_idx )) ;
    waypt_traj_px = reshape( navdata_line([14,15, 18,19, 22,23, 26,27, 31,32]), 2, 5)' / resolution ; 
    tgt_wx_px = waypt_traj_px(2,1) ; % 2nd wpt
    tgt_wy_px = waypt_traj_px(2,2)  ; % 
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

    % === filter subgoals based on its orientation if necessary
    sg_vidx = 1:num_sgs ;
    if angle_filter == true
        sgs_quat = sgs_xyzq_corrected_px(:,4:end) ;
        qdist = zeros(num_sgs,1) ;
        for ii =1:num_sgs
            qdist(ii) = quat_distance(sgs_quat(ii,:), [1,0,0,0]) ;
        end
        sg_vidx = find( qdist < pi/2 );
        %sgs_xyzq_corrected_px = sgs_xyzq_corrected_px(vidx, : ) ;
    end


    %% === Step 5: plotting  === %%  
    arrow_step = 1; arrow_scale = 20;

    fig = figure(1); clf;
    fig.Position = [2800, 300, 1600, 780] ;
    tmain = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tmain, sprintf('Bag ID: %s', navtime_id),'FontWeight','bold','FontSize',14, 'Interpreter','none');
    
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
    imshow( costmap_i8 );  % Show potential field as a heatmap
    hold on;
    
    drawSubGoalPoses(sgs_xyzq_corrected_px(sg_vidx,:), rx, ry, 2) ; % draw SLAM SGs

    plot( rx + waypt_traj_px(:,1) * 100 , ry + waypt_traj_px(:,2) * 100, 'ys', 'MarkerFaceColor','y') ;
%    plot( sgs_px_cands(:,1), sgs_px_cands(:,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b' ) ;
    plot( rx, ry, 'g>', 'MarkerSize', 15, 'MarkerFaceColor', 'g' ) ;
    plot( rx + old_sg_px(1), ry + old_sg_px(2),  'mo', 'MarkerSize', 8 , 'MarkerFaceColor', 'm') ;

    colorbar; 
    title( sprintf('Data idx: %d / %d ,  # Collected Data: %d ',data_idx, num_data, data_cnt)) ;
    hold on;

    %    dist_to_sg_cands = sqrt( (sgs_px_cands(:,1) - gx).^2 + (sgs_px_cands(:,2) - gy).^2 ) ;
    new_sg_idx = sg_idx; %find(dist_to_sg_cands == min(dist_to_sg_cands)) ;
    new_sg_xyzq_px = sgs_xyzq_corrected_px(new_sg_idx, :) ;
    new_sg_xyzq_m  = new_sg_xyzq_px ;
    new_sg_xyzq_m(1:3)  = new_sg_xyzq_m(1:3) * resolution ;
    plot( new_sg_xyzq_px(1) + rx, new_sg_xyzq_px(2) + ry, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'm' ) ;

    lgd = legend('Pred Waypts', 'SG(SLAM)', 'Robot', 'SG(pred)', 'SG(corr)', 'Location', 'NW')
    lgd.Color = [0.8 0.8 0.8];
    set(gca,'YDir','normal');

    if angle_filter
        legPos = get(lgd, 'Position')
        XL = xlim; YL = ylim;
        
        % Convert legend position to data coordinates
        x_arrow = XL(1) + (XL(2)-XL(1)) * (legPos(1) + legPos(3) + 0.03);
        y_arrow = YL(1) + (YL(2)-YL(1)) * (legPos(2) + legPos(4)/2);
        
        % ---- Quiver arrow: perfectly horizontal ----
        arrow_length = (XL(2)-XL(1)) * 0.15;   % 15% of width
        quiver(x_arrow, y_arrow, arrow_length, 0, ...
               'LineWidth', 3, 'Color', 'r', 'MaxHeadSize', 2);
        
        % ---- Text to the right of arrow ----
        text(x_arrow + (XL(2)-XL(1))*0.01, y_arrow + 5, ...
             'angle filter is on', ...
             'FontWeight', 'bold', 'FontSize', 12);
    end

    %% == 6. Data collection from expert's choice == %%
    [gx, gy, button] = ginput(1) ;
    if button == 3
        disp('skipping 10 frames \n');
        step_idx = step_idx + 10;
        fprintf('step_idx : %d ', step_idx)
        continue;
    else
        step_idx = step_idx + 1;
        if (gx <0 | gx > map_size_px | gy < 0 | gy > map_size_px)
            continue;  % if the not qualitifed to be a collision data. Don't bother
        end
    end

error(-1)

    sgs_xy_px = sgs_xyzq_corrected_px(sg_vidx,1:2)' + rx ;  assert (rx == ry) ;

    mydist = sqrt( ( sgs_xy_px(1,:) - gx ).^2 + ( sgs_xy_px(2,:) - gy ).^2 ) ;
    minidx_of_sgvidx = find(mydist == min(mydist) ) ;
    new_sg_idx = sg_vidx( minidx_of_sgvidx ) ;

    new_sg_xyzq_px = sgs_xyzq_corrected_px(new_sg_idx, :) ;
    plot( rx + new_sg_xyzq_px(1), rx + new_sg_xyzq_px(2),  'yx', 'MarkerSize', 12 , 'LineWidth', 3) ;
    

    set(gcf, 'pointer', 'crosshair'); 
    set(gcf, 'pointer', 'arrow');
    P0_m = [rx, ry] * resolution ; 
    P2_m = [new_sg_xyzq_m(1), new_sg_xyzq_m(2)] + P0_m ;

    % Compute the corrected waypoints
    P0m_xyzq = [0 0 0 1 0 0 0] ;
    P2m_xyzq = [new_sg_xyzq_px(1:3) * resolution, new_sg_xyzq_px(4:end) ] ;
    
    P1m_xyzq = makeP1Between(P0m_xyzq, P2m_xyzq, alpha=0.5) ;
    out_waypts_m = planKinodynamicPath(P0m_xyzq, P1m_xyzq, P2m_xyzq, v_max, w_max, FPS) ;
    out_waypts_m = out_waypts_m(1:ws:end,:)  ; % waypoint spacing 3
    out_waypts_m = out_waypts_m(2:6,:) ;    % next 5 steps
    out_waypts_px = out_waypts_m ; out_waypts_px(:,1:2) = out_waypts_px(:,1:2) / resolution + map_size_px/2  ;

    %[out_waypts_px, target_waypoint_px] = sample_corrected_waypoints(P0_xyzq, P2_xyzq, resolution, fps, ws) ;
    %plot( out_waypts(:,1) + rx, out_waypts(:,2) + rx, 'gs', 'MarkerFaceColor', 'g') ;
    drawPoses2D(out_waypts_px) ; 

    fn = names{new_sg_idx} ;
    global_sg_idx = str2double(regexp(fn, '\d+', 'match', 'once')) ;
    target_subgoal_idxs = [new_sg_idx, global_sg_idx] ;  % sg idx of /topomap folder, global sg idx in /synced folder
    context_idxs = [data_idx-context_len*ws:ws:data_idx ] ;
    %nc = length(context_idxs) ;
    % (1) save prev robot pose 
    % robot pose w.r.t curr robot pose
    wHr = nav_o2Hb(:,:,context_idxs) ;
    rcHr = zeros(4,4,context_len) ; 
    rcHr(:,:,end) = wHr(:,:,end) ;

    %% ================================================================= %%
    %   save the collision dataset
    %% ================================================================= %%

    if save_data 
        
        %% == 7. Store data == %%
        out_dir = sprintf('%s/data%05d',colldata_dir, data_cnt) ;
        if (isdir(out_dir))
            rmdir(out_dir, 's') ;
        end
        mkdir(out_dir) ;

        % curr data idx, target sg idx, global target sg idx (in topomap)
        fid = fopen( sprintf('%s/data_sg_idx.txt', out_dir), 'w') ; 
        fprintf(fid, '%d %d %d', data_idx-1, new_sg_idx-1, global_sg_idx);
        fclose(fid) ;

        fid = fopen( sprintf('%s/pose_context_m.txt', out_dir) , 'w') ;
        for ii=1:size(wHr,3)  %context_len
            rcHr(:,:,ii) = inv(wHr(:,:,end)) * wHr(:,:,ii) ;
%This line was weird  why * resolution ? 
            x_prev_m = rcHr(1,4,ii) ; % m
            y_prev_m = rcHr(2,4,ii) ;
            q = htm_to_quat( rcHr(:,:,ii) ) ; 
            fprintf( fid, '%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', [ x_prev_m y_prev_m 0 q(:)']' ) ; % x y z qw qx qy qz
        end
        fclose(fid);
    
        % (2) save context rgb and depth images
        for cidx = 1 : length(context_idxs)
            src_rgb = sprintf('%s/rgb%05d.png', sync_metadata_dir, context_idxs(cidx)-1 )  ; % python conv
            src_dep = sprintf('%s/depth%05d.png', sync_metadata_dir, context_idxs(cidx)-1 ) ; % python conv
            copyfile(src_rgb, sprintf('%s/', out_dir) ) ;
            copyfile(src_dep, sprintf('%s/', out_dir) ) ;
        end
    
        % (2)-1 save sg rgb and depth images
        copyfile(rgb_sg_file, sprintf('%s/rgb_sg.png', out_dir)) ;
        copyfile(depth_sg_file, sprintf('%s/depth_sg.png', out_dir));
        
        % (3) save costmap
        fid = fopen( sprintf('%s/costmap_info.txt', out_dir), 'w' ) ;
        fprintf(fid, 'resolution, map_size_px \n %f %d ', resolution, map_size_px) ;
        writematrix( costmap_i8, sprintf('%s/costmap_i8.txt', out_dir ) ) ;
    
        % (4) save corrected waypoint
        fid = fopen( sprintf('%s/corrected_waypoints_m.txt', out_dir), 'w' ) ; % x,y,z, qw,qx,qy,qz
        fprintf( fid, '%6.4f %6.4f %6.4f %6.4f \n', out_waypts_m(:,[1,2,4,end])' ) ;  
        fclose(fid) ;
    
        % (5) save pred waypoints
        pred_waypts_m = reshape( navdata_line(14:end), 4, context_len)' ;
        fid = fopen( sprintf('%s/pred_waypoints_m.txt', out_dir), 'w') ;
        fprintf( fid, '%6.4f %6.4f %6.4f %6.4f \n', pred_waypts_m') ;
        fclose(fid) ;
    
        % (6) save context idx
        fid = fopen( sprintf('%s/context_index.txt', out_dir), 'w') ;
        fprintf(fid, '%d ', context_idxs-1); fprintf(fid, '\n');
        fclose(fid) ;
    
        % (7) save sg_idx
        % fid = fopen( sprintf('%s/subgoal_index.txt', out_dir), 'w') ;
        % fprintf(fid, '%d ', subgoal_idxs); fprintf(fid, '\n');
        % fclose(fid) ;

        % dx = gx - rx ;   % wpt1  should heading to wpt2
        % dy = gy - ry ;
        % theta = atan2(dy, dx) ;  % radians
        % half_theta = theta / 2 ;
        % q = [cos(half_theta), zeros(size(theta)), zeros(size(theta)), sin(half_theta)] ;
        % q = q / norm(q) ; % subgoal orient
        
        % (7) save SGs

        fid = fopen( sprintf('%s/old_subgoal_m.txt', out_dir), 'w') ; % old; 
        fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', old_sg_px(1) * resolution, old_sg_px(2) * resolution, old_sg_px(3), old_sg_px(4) ) ;
        fclose(fid) ;
    
        fid = fopen( sprintf('%s/new_subgoal_m.txt', out_dir), 'w') ; % corrected
        fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', new_sg_xyzq_px(1) * resolution, new_sg_xyzq_px(2) * resolution ...
                                                  , new_sg_xyzq_px(4), new_sg_xyzq_px(end) ) ;
        %fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', dx*resolution, dy*resolution, q(1), q(4) ) ;
        
        fclose(fid) ;
    end

    data_cnt = data_cnt + 1 ;

end