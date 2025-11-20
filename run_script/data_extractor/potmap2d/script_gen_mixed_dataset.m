% gen coll and noncoll dataset


colldata_folder = '/media/mydata/former_datasets/colldata/colldata-all' ;
noncolldata_folder = '/media/mydata/former_datasets/colldata/noncolldata-11K' ;

input_bag_paths = dir( sprintf('%s/bag_*', colldata_folder) ) ;
out_paths = '/media/mydata/former_datasets/colldata/mixed-tmp' ;

% 1st copy all data from colldata-all to 
data_cnt = 0;
for idx=1:length(input_bag_paths)
    tmp_path = sprintf('%s/%s', input_bag_paths(idx).folder, input_bag_paths(idx).name ) ;
    datasets = dir(sprintf('%s/data*', tmp_path) ) ;

    wd = cd;
    cd(colldata_folder) ;
    for ii=1:length(datasets)
        src_path = sprintf('%s/%s', datasets(ii).folder, datasets(ii).name ) ;
        tgt_path = sprintf('%s/data%05d', out_paths, data_cnt ) ;
        cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
        system(cmd) ;
        data_cnt = data_cnt + 1;
    end
    display(data_cnt)
end



colldata =  dir(sprintf('%s/data*',colldata_folder) ) ;
noncolldata = dir(sprintf('%s/data*', noncolldata_folder) ) ;

% 1st copy colldata
data_cnt = 0 ; 
for idx=1:length(colldata)
    data_cnt
    src_path = sprintf('%s/%s', colldata(idx).folder, colldata(idx).name ) ;
    tgt_path = sprintf('%s/data%05d', out_paths, data_cnt) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
    system(cmd) ;
    data_cnt = data_cnt + 1;
end

% 2nd copy noncolldata
for idx=1:2500 %length(noncolldata)
    data_cnt
    src_path = sprintf('%s/%s', noncolldata(idx).folder, noncolldata(idx).name ) ;
    tgt_path = sprintf('%s/data%05d', out_paths, data_cnt) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path);
    system(cmd) ;
    data_cnt = data_cnt + 1;
end




