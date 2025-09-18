function [ dataset ] = get_noncoll_dataset( bag_path, config, seed_ratio )

    sync_data_path = sprintf('%s/%s/synced', bag_path.folder, bag_path.name ) ;
    rgb_files = dir(sprintf('%s/rgb*.png',sync_data_path)) ;
    num_frames = length( rgb_files )  ;
    num_seeds = int32(seed_ratio * num_frames) ;

    last_frame_idx = num_frames - 1;
    ws = config.datasets.former.waypoint_spacing ;
    max_frame_dist = config.distance.max_frame_dist ;
    max_goal_dist = ws * max_frame_dist ;
    context_size = config.context_size ;
    len_traj_pred = config.len_traj_pred ;
    
    sidx = context_size * ws ;
    eidx = (num_frames - max_frame_dist) - sidx  ;
    assert(eidx > num_seeds) ;
    seeds = randperm(eidx, num_seeds) + sidx ;
    %seeds = randi([ context_size * ws, num_frames - max_frame_dist ], 1, num_seeds) ;

    odom_file = sprintf('%s/sync_odom.txt', sync_data_path) ;
    [pose_raw, xy, wHb] = load_pose_data( odom_file ) ;

    dataset = struct ;
    for idx=1:length(seeds)
        curr_idx = seeds(idx) ;
        context_idx = curr_idx - [context_size:-1:0] *ws ;
        assert(context_idx(1) >= 0) ;
        % sample goal
        max_frame_goal_dist = min( max_goal_dist, last_frame_idx - curr_idx ) ;
        goal_offset = randi( [ws, max_frame_goal_dist] , 1 ) ;
        goal_idx = curr_idx + goal_offset ;
        assert(goal_idx <= last_frame_idx) ;
        
        waypoint_idx = curr_idx + [1:len_traj_pred] * ws ;
        % compute context pose
        wHb_context = wHb(:,:, context_idx+1 ) ; % matlab idx conv +1
        wHb_sg = wHb(:,:,goal_idx+1 ) ; % matlab idx conv +1
        pose_context_m = zeros(context_size+1, 7) ;
        for ii=1:(context_size+1)
            b0Hb(:,:,ii) = inv(wHb_context(:,:,end)) * wHb_context(:,:,ii) ;
            quat = htm_to_quat( b0Hb(:,:,ii) ) ;
            pose_context_m(ii,:) = [ b0Hb(1:3,4,ii); quat(:) ]' ; 
        end
        b0Hb_sg = inv( wHb_context(:,:,end) ) * wHb_sg ;
        q_sg = htm_to_quat(b0Hb_sg) ;
        sg_pose = [ b0Hb_sg(1:2,4); q_sg(1); q_sg(end) ]' ;
    
        % compute waypoints
        b0Hb = [] ;
        wHb_waypoints = wHb(:,:,waypoint_idx+1) ;
        corrected_waypoints_m = zeros(len_traj_pred, 4) ; % x y qw qz
        for ii=1:len_traj_pred
            b0Hb(:,:,ii) = inv(wHb(:,:,curr_idx+1)) * wHb_waypoints(:,:,ii) ;
            quat = htm_to_quat( b0Hb(:,:,ii) ) ;
            corrected_waypoints_m(ii,:) = [b0Hb(1:2,4,ii); [quat(1) quat(4)]']'  ;
        end
    
        dataset(idx).curr_idx = curr_idx ;
        dataset(idx).goal_idx = goal_idx ;
        dataset(idx).context_idx = context_idx ;
        dataset(idx).waypoint_idx = waypoint_idx ;
        % compute waypoints 
        dataset(idx).corrected_waypoints_m = corrected_waypoints_m ;
        dataset(idx).pose_context_m = pose_context_m ;
        dataset(idx).sg_pose = sg_pose ;
        dataset(idx).bag_id = bag_path.name ;
    end


end