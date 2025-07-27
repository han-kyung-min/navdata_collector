
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

% init pose
px0 = odom(1,1) ;
py0 = odom(1,2) ;
orient = odom(1, 4: end) ; 
theta0 = orient(3)  ;
[hx0, hy0] = pol2cart(theta0, 1) ;

euc_dist = zeros(1,length(odom)) ;
ang_dist = zeros(1,length(odom)) ;

pose3 = [odom(:,1), odom(:,2), odom(:,end)] ;

% === Loop over 3000 datasets ===

output_dir = 'costmaps';  % Where to save .mat or .png files
mkdir(output_dir);

for data_idx = 1:num_data
    % === Construct file paths ===
    scandata = load(sprintf('%s/scan%05d.txt', sync_metadata_dir, data_idx-1) ) ;
    ranges = scandata(:,2) ;
    angles = scandata(:,1) ;

    % build costmap
    [costmap_i8, costmap_u8] = build_costmap( ranges, angles) ;

    % === Save the costmap ===
    %out_path = fullfile(output_dir, sprintf('costmap_%04d.png', i));
    %imwrite(final_costmap, gray(256), out_path);

    % Assume final_costmap is already loaded and is of size (H x W)
    [H, W] = size(costmap_i8) ;
    [x, y] = meshgrid(1:W, 1:H)  ;
    
    % Threshold: consider pixels with cost > 90 as obstacles
    obstacle_mask = costmap_i8 > 90;
    
    % Get obstacle pixel coordinates
    [obs_y, obs_x] = find(obstacle_mask);  % row, col indices of obstacles
    
    % Initialize repulsive vector field
    Ux = zeros(H, W);
    Uy = zeros(H, W);
    
    epsilon = 1e-5;
    
    % Compute repulsive potential for each free-space cell
    for i = 1:H
        for j = 1:W
            if ~obstacle_mask(i, j)
                % Compute distances to all obstacle pixels
                dx = j - obs_x;
                dy = i - obs_y;
                dist_sq = dx.^2 + dy.^2 + epsilon;
                [~, idx] = min(dist_sq);  % nearest obstacle
    
                % Set repulsive vector pointing outward
                Ux(i, j) = dx(idx) / dist_sq(idx);
                Uy(i, j) = dy(idx) / dist_sq(idx);
            end
        end
    end
    
    % Plot the costmap
    figure;
    imagesc(costmap_i8);  % Display: white = free, black = obstacle
    colormap(gray);
    axis equal;
    hold on;
    
    plot( H/2, W/2, 'mo', 'MarkerSize',16, 'MarkerFaceColor', 'c') ;

    % Plot the repulsive field
    
    % Control arrow density and size
    step = 2;
    quiver(x(1:step:end,1:step:end), ...
           y(1:step:end,1:step:end), ...
           Ux(1:step:end,1:step:end), ...
           Uy(1:step:end,1:step:end), ...
           'r', 'AutoScale', 'on', 'AutoScaleFactor', 1.6);
    
    title('Costmap w/ potential field');
    


end