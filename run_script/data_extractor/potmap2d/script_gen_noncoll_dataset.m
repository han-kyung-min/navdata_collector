% generate ordinary dataset from a bag file

clear all; close all; clc;

devgru_dir = '/home/hankm/python_ws/viznav/depth-nav' ;
config_file = sprintf('%s/config/depth_nav.yaml',devgru_dir) ;
config = ReadYaml(config_file) ;

%data_path = '/media/mydata/former_datasets/colldata/bag_2025-08-19-12-05-34' ;
% gen dataset subfolders
data_root_path = '/media/mydata/former_datasets/former_all' ;
bag_paths = dir( sprintf('%s/bag_*', data_root_path) ) ;

%data_path = '/media/mydata/former_datasets/colldata/bag_2025-08-19-12-10-35' ;  % robot in idle
%sync_data_path = sprintf('%s/synced', data_path) ;

dataset = [] ;
for idx =1:length(bag_paths)
    bag_path = bag_paths(idx) ;
    try
        [ data ] = get_noncoll_dataset( bag_path, config, 0.2 ) ;
        dataset = [dataset, data] ;
    catch
    end
end

num_dataset = length(dataset) / 2;
% copy to non_colldata 
out_folder = '/media/mydata/former_datasets/colldata/mixed-25K' ;
begin_offset = 5667 ; %num_dataset ;

rng(42);  
myidx_list = [1:num_dataset] ; %randperm(num_dataset, 5700) ;

for tmpidx=1 : length(myidx_list) %num_dataset 
    fprintf('\rProcessing <%d>th data %%', tmpidx)
    out_dataidx = tmpidx + begin_offset ;
    dataset_path = sprintf('%s/data%05d', out_folder, out_dataidx) ;

    if isdir(dataset_path)
        rmdir(dataset_path, 's') ;
    end
    mkdir(dataset_path) ;

    idx = myidx_list(tmpidx) ;
    goal_idx = dataset(idx).goal_idx ;
    context_idx = dataset(idx).context_idx ;
    src_data_path = sprintf('%s/%s/synced',data_root_path, dataset(idx).bag_id) ;

    % copy image files
    for ii=1:length(context_idx)
        src_rgb_file = sprintf('%s/rgb%05d.png', src_data_path, context_idx(ii) ) ;
        tgt_rgb_file = sprintf('%s/rgb%05d.png', dataset_path, context_idx(ii)) ;
        src_dep_file = sprintf('%s/depth%05d.png', src_data_path, context_idx(ii) ) ;
        tgt_dep_file = sprintf('%s/depth%05d.png', dataset_path, context_idx(ii)) ;
        src_scan_file = sprintf('%s/scan%05d.txt', dataset_path, context_idx(ii)) ;
        tgt_scan_file = sprintf('%s/scan%05d.txt', dataset_path, context_idx(ii)) ;
        system(sprintf('cp -rf %s %s', src_rgb_file, tgt_rgb_file)) ;
        system(sprintf('cp -rf %s %s', src_dep_file, tgt_dep_file)) ;
        %system(sprintf('cp -rf %s %s', src_scan_file, tgt_scan_file)) ;
    end
    % cp sg img files
    src_rgb_sg_file = sprintf('%s/rgb%05d.png', src_data_path, goal_idx) ;
    tgt_rgb_sg_file = sprintf('%s/rgb_sg.png', dataset_path) ;
    src_dep_sg_file = sprintf('%s/depth%05d.png', src_data_path, goal_idx) ;
    tgt_dep_sg_file = sprintf('%s/depth_sg.png', dataset_path) ;

    system( sprintf('cp -rf %s %s', src_rgb_sg_file, tgt_rgb_sg_file) ) ;
    system( sprintf('cp -rf %s %s', src_dep_sg_file, tgt_dep_sg_file) ) ;

    % gen subgoal_m file
    fid = fopen( sprintf('%s/old_subgoal_m.txt', dataset_path), 'w' );
    fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', dataset(idx).sg_pose  ) ;
    fclose(fid) ;
    
    fid = fopen( sprintf('%s/new_subgoal_m.txt', dataset_path), 'w' );
    fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', dataset(idx).sg_pose  ) ;
    fclose(fid) ;

    % gen pose_context
    fid = fopen( sprintf('%s/pose_context_m.txt', dataset_path), 'w') ;
    for ii=1:(config.context_size+1)
        fprintf(fid, '%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', dataset(idx).pose_context_m(ii,:) );
    end
    fclose(fid) ;

    % gen waypoints
    fid = fopen( sprintf('%s/corrected_waypoints_m.txt', dataset_path), 'w') ;
    for ii=1:config.len_traj_pred
        fprintf(fid, '%6.4f %6.4f %6.4f %6.4f\n', dataset(idx).corrected_waypoints_m(ii,:) );
    end
    fclose(fid) ;
    
    fid = fopen( sprintf('%s/context_index.txt', dataset_path), 'w') ;
    fprintf(fid, '%d %d %d %d %d %d\n', context_idx );
    fclose(fid);

    fid = fopen( sprintf('%s/sg_idx.txt', dataset_path), 'w') ;
    fprintf(fid, '%d \n', dataset(idx).goal_idx) ;
end



% 
% for idx=0:200
%     tmp_path = sprintf('/media/mydata/former_datasets/colldata/colldata_all/data%05d',idx);
%     coll_file = sprintf('%s/colldata',tmp_path) ; 
%     system(sprintf('touch %s', coll_file)) ;
% end
% 










