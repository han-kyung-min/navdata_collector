
% nav res viewer
clear all; close all; clc;

topomap_dir = '/home/hankm/python_ws/viznav/depth-nav/deployment/topomaps/T1-1';
%base_topomap_path = '/media/data/results/navdata_collector/topomap/processed/2026-01-22-16-55/bag_2026-01-22-16-55-27/';

map_png_path = sprintf('%s/slam_map.png',topomap_dir) ;
map_yaml_path= sprintf('%s/slam_map.yaml',topomap_dir);

map_png = imread(map_png_path) ;
map_yaml = ReadYaml(map_yaml_path) ;
xo_m = map_yaml.origin{1};
yo_m = map_yaml.origin{2};
tho = map_yaml.origin{3};
res = map_yaml.resolution ;

[H, W] = size(map_png) ; 

col0 = floor((0 - xo_m) / res);
row0 = (H - 1) - floor((0 - yo_m) / res); 
xo_px = col0 + 1;
yo_px = row0 + 1;


% draw topomap
topop_m2b = load( sprintf('%s/topo_tf_m2b.txt', topomap_dir) ) ;
num_nodes = size(topop_m2b,1) ;
topomap_m = zeros(num_nodes, 3) ;
topomap_px = zeros(num_nodes, 2) ;
for idx=1:num_nodes
    q = topop_m2b(idx, end-4:end);
    x = topop_m2b(idx,5) ;
    y = topop_m2b(idx,6) ;
    %quat_to_htm([q(end), q(1:3)]) ;
    %htm(1:2,4) = xy ;
    topomap_m(idx,1:2) = [x,y];
    topomap_px(idx,1) = floor((x - xo_m) / res) ;
    topomap_px(idx,2) = (H - 1) - floor((y - yo_m) / res); 
end

figure(1); hold on;
plot(topomap_px(1:4:end,1), topomap_px(1:4:end,2), 'co') ;

% display nav res on the map

%base_nav_path = '/media/results/navdp/2026-01-26-11-13' ;
base_nav_path = '/media/results/nomad/2026-02-01-11-25' ;
traj_path = sprintf('%s/traj', base_nav_path) ;

m2b_files = dir(fullfile(traj_path, "m2b_*.txt"));
num_poses = length(m2b_files);

pxs = nan(1, num_poses);
pys = nan(1, num_poses);

nav_traj_m = zeros(num_poses,2) ;
for idx = 1:num_poses
    m2b_path = fullfile(traj_path, sprintf("m2b_%05d.txt", idx));
    if ~isfile(m2b_path)
        continue;
    end

    m2b = load(m2b_path);
    x = m2b(1);
    y = m2b(2);
    nav_traj_m(idx,:) = [x, y];
    % Map (meters) -> pixel (col,row), yaw=0
    col = floor((x - xo_m) / res) ;
    row = (H - 1) - floor((y - yo_m) / res); 

    px = col + 1;
    py = row + 1;

    if px >= 1 && px <= W && py >= 1 && py <= H && isfinite(px) && isfinite(py)
        pxs(idx) = px;
        pys(idx) = py;
    end
end

valid = ~isnan(pxs) & ~isnan(pys);
map_png_rot = rot90(map_png, -1) ;



h1= figure(1) ;clf;
imshow(map_png_rot);  hold on;
%plot(xo_px, yo_px, 'r+') ;

pxs2 = H - pys + 1;
pys2 = pxs;

plot(pxs2(valid), pys2(valid), 'b.', 'MarkerSize', 8);

i0 = find(valid, 1, 'first');
i1 = find(valid, 1, 'last');
if ~isempty(i0)
    plot(pxs2(i0), pys2(i0), 'go', 'MarkerFaceColor','g', ...
        'MarkerEdgeColor','m', 'MarkerSize', 12);
end
if ~isempty(i1)
    plot(pxs2(i1), pys2(i1), 'r*', 'MarkerFaceColor','r', 'MarkerSize', 16);
end

topo_pxs2 = H - topomap_px(:,2) + 1;
topo_pys2 = topomap_px(:,1);

% draw goal node
% plot(topo_pxs2(end), topo_pys2(end), 'co', ...
%      'MarkerFaceColor', [0.9290, 0.6940, 0.1250], 'MarkerEdgeColor','m', 'MarkerSize', 12);

legend('Nav traj', 'Start pos', 'Final pos', 'Goal pos', 'Location', 'SW') ;
trav_dist = sqrt(sum( (nav_traj_m(end,:) - nav_traj_m(1,:)).^2 ) )  ;
exportgraphics(gca, 'nomad_res.png', 'Resolution', 300);
exportgraphics(gca, 'nomad_res.pdf', 'Resolution', 300);

h2 =figure(2);
imshow(map_png_rot); hold on;
% draw topology nodes
% sg_space = 8 ;
idx = 1:5:length(topo_pxs2);

x = topo_pxs2(idx);
y = topo_pys2(idx);
% Draw nodes
plot(x, y, 'o', ...
     'MarkerFaceColor','c', ...
     'MarkerEdgeColor','m', ...
     'MarkerSize',8);

plot(x(1), y(1), 'o', ...
     'MarkerFaceColor','g', ...
     'MarkerEdgeColor','m', ...
     'MarkerSize',12);

plot(x(end), y(end), 'o', ...
     'MarkerFaceColor', [0.9290, 0.6940, 0.1250], ...
     'MarkerEdgeColor','m', ...
     'MarkerSize',12);


% Arrow vectors (dx, dy)
dx = diff(x);
dy = diff(y);

% Draw arrows
quiver( x(1:end-1), y(1:end-1), ...
        dx, dy, ...
        0, ...                    % 0 = no auto scaling (IMPORTANT)
        'Color','r', ...
        'LineWidth',1.0, ...
        'MaxHeadSize',0.8 );

legend('Topology nodes','Start node', 'Goal node', 'Node to node direction',  'Location','southwest');
hold off;
exportgraphics(gca, 'topomap.png', 'Resolution', 300);
exportgraphics(gca, 'topomap.pdf', 'Resolution', 300);