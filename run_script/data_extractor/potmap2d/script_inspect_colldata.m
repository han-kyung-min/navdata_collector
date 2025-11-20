
% colldata inspector

clear all; close all; clc;

% source folder
pkg_dir = fileparts( fileparts( fileparts(pwd) ) ) ;
config_file = sprintf('%s/param/navdata_collector.yaml',pkg_dir);
config = ReadYaml(config_file) ;
proc_dir = '/media/results/navdata_collector/colldata/processed' ;

colldata_dir = sprintf('/media/mydata/former_datasets/colldata/colldata-all');
colldata = dir(sprintf('%s/data*', bag_dir) ) ;

h= figure('Color','w','Units','normalized','Position',[0.02 0.2 0.5 0.3]); % wide canvas
    
for idx=8:length(colldata)
    
    depth_img_files = dir(sprintf('%s/%s/depth*.png', colldata(idx).folder, colldata(idx).name )) ;
    rgb_img_files = dir(sprintf('%s/%s/rgb*.png', colldata(idx).folder, colldata(idx).name )) ;
    
    depth_imgs = {} ;
    rgb_imgs = {};
    for ii=1:7
        depth_imgs{ii} = imread( sprintf('%s/%s',depth_img_files(ii).folder, depth_img_files(ii).name ) ) ;
        rgb_imgs{ii} = imread( sprintf('%s/%s', rgb_img_files(ii).folder, rgb_img_files(ii).name )) ;
    end

    depth_sg_file = sprintf('%s/%s/depth_sg.png', colldata(idx).folder, colldata(idx).name) ;
    rgb_sg_file = sprintf('%s/%s/rgb_sg.png', colldata(idx).folder, colldata(idx).name) ;

    h; clf;

    tl = tiledlayout(2,7,'TileSpacing','compact','Padding','compact');
    title(tl, colldata(idx).name) ;

    % --- Top row: RGB images ---
    for k = 1:7
        nexttile;
        imshow(rgb_imgs{k});
        title(sprintf('RGB %d',k),'FontWeight','normal');
        axis image off;
    end
    
    % --- Bottom row: Depth images ---
    for k = 1:7
        nexttile;
        % If depth is uint16 in millimeters, choose a sensible display range.
        % Example: clip to [0.3m, 3.0m] => [300, 3000] mm.
        if isa(depth_imgs{k},'uint16')
            imshow(depth_imgs{k}, [300 3000]);     % adjust to your sensor range
        else
            % If depth is in meters (float), display 0.3–3.0 m
            imshow(depth_imgs{k}, [0.3 3.0]);
        end

        [r, c] = find( depth_imgs{k} < 450 ) ;
        hold on;
        plot(c, r, 'r.'); hold off;

        colormap(gca, parula);                 % per-axes colormap
        axis image off;
        title(sprintf('Depth %d',k),'FontWeight','normal');
    end
    
    % Optional: one colorbar for all depth tiles (attach to last one)
    cb = colorbar(nexttile(14));
    cb.Layout.Tile = 'east';   % puts a single colorbar on the right side
    cb.Label.String = 'Depth';

    pause ;

end