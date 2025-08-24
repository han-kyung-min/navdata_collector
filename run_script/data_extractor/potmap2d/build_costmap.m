
function [costmap_i8, costmap_u8 ] = build_costmap(ranges, angles, cm_params)
    % === Costmap Parameters ===
    
    resolution = cm_params.resolution ;
    map_size_m = cm_params.map_size_m ;
    robot_radius_m = cm_params.robot_radius_m ;
    inflation_radius_m = cm_params.inflation_radius_m ;
    max_range_m = cm_params.max_range_m ;

    map_size_px = round(map_size_m / resolution);
    U = -1;  % 128;
    O = 100; %% 255;

    roi_front_m = 1.0 ; assert( roi_front_m < map_size_m ) ;
    roi_back_m = 0.25 ; assert( roi_back_m <= roi_front_m) ;

    robot_radius_px = round(robot_radius_m / resolution);
    inflation_radius_px = round(inflation_radius_m / resolution);

    se = strel('disk', inflation_radius_px);
    half_map = map_size_px / 2;

    % === Convert scan to local frame ===
    angles = -angles ; % CCW scan (right --> front --> left )
    x_local = ranges .* cos(angles);  
    y_local = ranges .* sin(angles);

    % === Map scan to grid indices ===
    ix = round(x_local / resolution + half_map);
    iy = round(y_local / resolution + half_map);

    % === Initialize costmap with -1 (unknown) ===
    
    final_costmap = U * ones(map_size_px, map_size_px, 'int8') ;

    % === Identify valid indices ===
    valid = ix >= 1 & ix <= map_size_px & iy >= 1 & iy <= map_size_px ;
    ix = ix(valid);
    iy = iy(valid);

    % === Mark obstacles with 100 ===
    for k = 1:length(ix)
        final_costmap(iy(k), ix(k)) = O;
    end
% === Raycasting for all beams ===
    for k = 1:length(angles)
        theta = angles(k);

        % End of beam: either actual hit or max range
        if ~isnan(ranges(k)) && ranges(k) > 0 && ranges(k) < max_range_m
            r = ranges(k);
        else
            r = max_range_m;
        end

        % Convert to local coordinates
        x_local = r * cos(theta);
        y_local = r * sin(theta);

        % Grid coordinates
        x0 = round(half_map);
        y0 = round(half_map);
        x1 = round(x_local / resolution + half_map);
        y1 = round(y_local / resolution + half_map);

        % Raytrace to mark free space
        [rr, cc] = bresenham(y0, x0, y1, x1);
        rr = max(min(rr, map_size_px), 1);
        cc = max(min(cc, map_size_px), 1);

        for n = 1:length(rr)-1  % exclude last if obstacle
            if final_costmap(rr(n), cc(n)) == U
                final_costmap(rr(n), cc(n)) = 0;
            end
        end

        % Mark obstacle at the end (if range is valid)
        if ranges(k) > 0 && ranges(k) < max_range_m
            if y1 >= 1 && y1 <= map_size_px && x1 >= 1 && x1 <= map_size_px
                final_costmap(y1, x1) = O;
            end
        end
    end


    % === Obstacle Inflation with Decay ===
    obstacle_mask = final_costmap == 100;
    decayed_costmap = zeros(size(final_costmap), 'uint8');
    
    for r = 1:inflation_radius_px
        se_ring = strel('disk', r);
        ring = imdilate(obstacle_mask, se_ring);
        
        % Decay value (scaled from 100 to 1)
        decay_val = uint8(100 - round((r / inflation_radius_px) * 100));
    
        % Apply to all pixels except the core obstacle
        mask = ring & final_costmap ~= 100;
        decayed_costmap(mask) = max(decayed_costmap(mask), decay_val);  % take max decay if overlapping
    end


    % === Merge decay onto free space ===
    free_mask = final_costmap == 0;
    final_costmap(free_mask) = max( double(final_costmap(free_mask)), double(decayed_costmap(free_mask)) ) ;
    final_costmap = int8(final_costmap) ;

    % === Add robot footprint as cost 50 ===
    % [X, Y] = meshgrid(1:map_size, 1:map_size);
    % dist = sqrt((X - half_map).^2 + (Y - half_map).^2);
    % footprint_mask = dist <= robot_radius_px;
    % final_costmap(footprint_mask & final_costmap ~= O) = 50;

    costmap_i8 = final_costmap ;
    costmap_u8 = remap_costmap(final_costmap) ;
    


    % imshow(costmap_255)
    % hold on; plot(map_size/2, map_size/2, 'ro')
    % 
    % imagesc(mapped_costmap);  % Visualize the result
    % colormap('jet'); colorbar;

end
