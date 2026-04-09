% gen coll and noncoll dataset

clear all; 
close all; 
clc;

colldata_folder = '/media/mydata/former_datasets/colldata/colldata-10K' ;
noncolldata_folder = '/media/mydata/former_datasets/colldata/noncolldata-30K' ;

input_bag_paths = dir( sprintf('%s/bag_*', colldata_folder) ) ;

base_out_dir = '/media/mydata/former_datasets/colldata/col-10K-fixed' ;
out_dirs = dir(sprintf('%s/noncol*', base_out_dir)) ;

rng(42);  

% gen shuffle list
noncolldata_dirs = dir( sprintf('/media/data/mydata/former_datasets/colldata/noncolldata-30K/data*') ) ;
p = randperm( length(noncolldata_dirs) ) ;
fid = fopen('/media/mydata/former_datasets/colldata/noncolldata-30K/shuffled_list.txt', 'w') ;
for idx=1:length(p)
    fprintf(fid, 'data%05d\n', p(idx)) ;
end
fclose(fid) ;

colldata_dirs = dir( sprintf('/media/data/mydata/former_datasets/colldata/colldata-10K/data*') ) ;
p = randperm( length(colldata_dirs) ) ;
fid = fopen('/media/mydata/former_datasets/colldata/colldata-10K/shuffled_list.txt', 'w') ;
for idx=1:length(p)
    fprintf(fid, 'data%05d\n', p(idx)) ;
end
fclose(fid) ;

noncolldata_list = readlines( sprintf('%s/shuffled_list.txt', noncolldata_folder) ) ;
colldata_list = readlines( sprintf('%s/shuffled_list.txt', colldata_folder) ) ;
noncolldata_list(end) = [] ;
colldata_list(end) = [] ;

num_colldata = 10000 ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   col_10K + noncol_2K
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
out_path = sprintf('%s/%s',base_out_dir, out_dirs(1).name) ;

% colldata first
data_cnt = 0 ;
for idx=1:num_colldata
    src_path = sprintf('%s/%s', colldata_folder, colldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

% do noncolldata
num_noncolldata = 2000 ;
data_cnt = num_colldata ;
for idx=1:num_noncolldata
    src_path = sprintf('%s/%s', noncolldata_folder, noncolldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path)  ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    col_10K + noncol_4K
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% colldata first
out_path = sprintf('%s/%s',base_out_dir, out_dirs(2).name) ;

data_cnt = 0 ;
for idx=1:num_colldata
    src_path = sprintf('%s/%s', colldata_folder, colldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

% do noncolldata
num_noncolldata = 4000 ;
data_cnt = num_colldata ;
for idx=1:num_noncolldata
    src_path = sprintf('%s/%s', noncolldata_folder, noncolldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path)  ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   col_10K + noncol_06K
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% colldata first
out_path = sprintf('%s/%s',base_out_dir, out_dirs(3).name) ;

data_cnt = 0 ;
for idx=1:num_colldata
    src_path = sprintf('%s/%s', colldata_folder, colldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

% do noncolldata
num_noncolldata = 6000 ;
data_cnt = num_colldata ;
for idx=1:num_noncolldata
    src_path = sprintf('%s/%s', noncolldata_folder, noncolldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path)  ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   col_8K + noncol_10K
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% colldata first
out_path = sprintf('%s/%s',base_out_dir, out_dirs(4).name) ;

data_cnt = 0 ;
for idx=1:num_colldata
    src_path = sprintf('%s/%s', colldata_folder, colldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path) ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end

% do noncolldata
num_noncolldata = 8000 ;
data_cnt = num_colldata ;
for idx=1:num_noncolldata
    src_path = sprintf('%s/%s', noncolldata_folder, noncolldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', out_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path)  ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end






% test noncolldata
noncolldata_test_path = '/media/data/mydata/former_datasets/colldata/noncolldata-test' ;
num_tot_noncolldata = length(noncolldata_list) ;
num_test_noncolldata = 500 ;
data_cnt = 0 ;
for idx=num_tot_noncolldata-num_test_noncolldata+1:num_tot_noncolldata
    src_path = sprintf('%s/%s', noncolldata_folder, noncolldata_list(idx) ) ;
    tgt_path = sprintf('%s/data%05d', noncolldata_test_path, data_cnt ) ;
    cmd = sprintf('cp -rf %s %s', src_path, tgt_path)  ;
    system(cmd) ;
    display(data_cnt) ;
    data_cnt = data_cnt + 1;
end




