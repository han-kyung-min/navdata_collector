%% select_weighted_samples_per_bag.m
% Select samples from data%05d folders across bag_* folders
% - Proportional to # of data* in each bag
% - At most 10 per bag
% - At least 1 per bag (if bag has >= 1 data folder)
% - Target TOTAL_SAMPLES (e.g., 500), but may be lower if impossible.

clear; clc;

%% USER SETTINGS
BASE_DIR      = '/media/data/mydata/former_datasets/colldata/colldata-all';
TOTAL_SAMPLES = 500;      % target total number of data%05d samples

rng(0);  % fixed random seed for reproducibility

%% FIND bag_* FOLDERS
bag_dirs = dir(fullfile(BASE_DIR, 'bag_*'));
bag_dirs = bag_dirs([bag_dirs.isdir]);

if isempty(bag_dirs)
    error('No bag_* folders found in %s', BASE_DIR);
end

num_bags = numel(bag_dirs);
fprintf('Found %d bag folders.\n', num_bags);

%% COLLECT data* FOLDERS IN EACH BAG
data_lists       = cell(num_bags, 1);  % dir() results of data* in each bag
num_data_per_bag = zeros(num_bags, 1); % number of data* folders per bag

for b = 1:num_bags
    bag_name = bag_dirs(b).name;
    bag_path = fullfile(BASE_DIR, bag_name);

    d_data = dir(fullfile(bag_path, 'data*'));
    d_data = d_data([d_data.isdir]);   % keep only directories

    data_lists{b}       = d_data;
    num_data_per_bag(b) = numel(d_data);

    fprintf('%s: %d data folders.\n', bag_name, num_data_per_bag(b));
end

total_available = sum(num_data_per_bag);
if total_available == 0
    error('No data* folders found under any bag_* folder.');
end

%% COMPUTE MAX/MIN PER BAG
% Each bag can contribute at least 1 (if it has any data) and at most 10,
% but we also cannot exceed its actual number of data folders.
max_per_bag = min(10 * ones(num_bags,1), num_data_per_bag);
min_per_bag = zeros(num_bags,1);
min_per_bag(num_data_per_bag > 0) = 1;   % at least 1 if bag has data

% Total possible minimum and maximum
min_possible_total = sum(min_per_bag);
max_possible_total = sum(max_per_bag);

fprintf('\nMin possible total = %d, Max possible total = %d\n', ...
        min_possible_total, max_possible_total);

if min_possible_total > TOTAL_SAMPLES
    warning(['Requested %d total samples, but min constraint requires %d. ', ...
             'Setting TOTAL_SAMPLES = %d.'], ...
             TOTAL_SAMPLES, min_possible_total, min_possible_total);
    TOTAL_SAMPLES = min_possible_total;
elseif max_possible_total < TOTAL_SAMPLES
    warning(['Requested %d total samples, but max constraint allows only %d. ', ...
             'Setting TOTAL_SAMPLES = %d.'], ...
             TOTAL_SAMPLES, max_possible_total, max_possible_total);
    TOTAL_SAMPLES = max_possible_total;
end

%% PROPORTIONAL ALLOCATION (BASED ON num_data_per_bag)
% Start from a proportional allocation ignoring constraints
weights = num_data_per_bag;
weights_sum = sum(weights);
raw_alloc = TOTAL_SAMPLES * (weights / weights_sum);

% Initial alloc = floor(raw_alloc), then enforce min/max
alloc = floor(raw_alloc);

% Enforce minimum 1 (for bags with data) and maximum 10 / available
for i = 1:num_bags
    if num_data_per_bag(i) == 0
        alloc(i) = 0;
        continue;
    end
    if alloc(i) < min_per_bag(i)
        alloc(i) = min_per_bag(i);
    end
    if alloc(i) > max_per_bag(i)
        alloc(i) = max_per_bag(i);
    end
end

current_total = sum(alloc);

%% ADJUST TO MATCH TOTAL_SAMPLES UNDER CONSTRAINTS
if current_total < TOTAL_SAMPLES
    % Need to add more samples while respecting max_per_bag
    remaining = TOTAL_SAMPLES - current_total;
    frac = raw_alloc - alloc;   % fractional parts

    % Bags already at max cannot increase
    full_mask = alloc >= max_per_bag;
    frac(full_mask) = -inf;

    while remaining > 0
        [val, idx] = max(frac);
        if ~isfinite(val)
            % No more room anywhere
            warning('Cannot reach desired TOTAL_SAMPLES under constraints. Using %d.', sum(alloc));
            break;
        end
        alloc(idx) = alloc(idx) + 1;
        remaining  = remaining - 1;

        % If we hit the max, mark as full
        if alloc(idx) >= max_per_bag(idx)
            frac(idx) = -inf;
        end
    end

elseif current_total > TOTAL_SAMPLES
    % Need to remove some while respecting min_per_bag
    to_remove = current_total - TOTAL_SAMPLES;
    frac = raw_alloc - alloc;   % fractional parts (more negative = less deserved)

    while to_remove > 0
        % Only consider bags whose alloc > min_per_bag
        can_reduce = alloc > min_per_bag;
        if ~any(can_reduce)
            warning('Cannot reduce alloc further without breaking min constraints.');
            break;
        end
        % Among reducible bags, remove from the one with smallest fractional part
        reducible_idx = find(can_reduce);
        [~, order] = sort(frac(reducible_idx), 'ascend'); % smallest first
        idx = reducible_idx(order(1));

        alloc(idx)   = alloc(idx) - 1;
        to_remove    = to_remove - 1;
    end
end

final_total = sum(alloc);
fprintf('\nFinal alloc per bag (sum = %d):\n', final_total);
T = table({bag_dirs.name}', num_data_per_bag, min_per_bag, max_per_bag, alloc, ...
          'VariableNames', {'BagFolder', 'NumDataFolders', 'MinAllowed', 'MaxAllowed', 'Allocated'});
disp(T);

%% RANDOMLY SELECT data* FOLDERS IN EACH BAG
selected = struct('bag_folder', {}, 'data_folder', {}, 'fullpath', {});

for b = 1:num_bags
    n_pick  = alloc(b);
    n_avail = num_data_per_bag(b);

    if n_pick <= 0 || n_avail == 0
        continue;
    end

    d_data = data_lists{b};
    idxs   = randperm(n_avail, n_pick);   % random unique data%05d indices

    for j = 1:n_pick
        d = d_data(idxs(j));

        entry.bag_folder  = bag_dirs(b).name;
        entry.data_folder = d.name;  % e.g., 'data00037'
        entry.fullpath    = fullfile(BASE_DIR, bag_dirs(b).name, d.name);

        selected(end+1) = entry; %#ok<SAGROW>
    end
end

fprintf('\nTotal selected samples: %d\n', numel(selected));

%% (OPTIONAL) SAVE LIST TO TEXT FILE
colldata_txt = fullfile('/media/data/mydata/former_datasets/colldata/colldata-test/test_colldata_list.txt') ;
fid = fopen(out_txt, 'w');
if fid == -1
    warning('Could not open %s for writing.', colldata_txt);
else
    for i = 1:numel(selected)
        fprintf(fid, '%s/%s/%s\n', BASE_DIR,selected(i).bag_folder, selected(i).data_folder);
    end
    fclose(fid);
    fprintf('Saved selected list to: %s\n', colldata_txt);
end

% mv test data file
filelines = readlines(colldata_txt) ;
filelines(end) = [] ;

for idx=1:length(filelines)
    src_file_path = filelines(idx) ;
    tgt_file_path = sprintf('/media/data/mydata/former_datasets/colldata/colldata-test/data%05d',idx-1) ;
    cmd = sprintf('mv %s %s', src_file_path, tgt_file_path ) ;
    system(cmd) ;
end
