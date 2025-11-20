function draw_depth_colormap( depth_img, near, far, th )

    N = 256;
    % ----- Plot your depth (scalar image) -----
    ax = gca;                         % or your specific axes
    imagesc(ax, depth_img, [near far]); % depth_m in meters
    axis(ax,'image'); axis(ax,'off');
    cb = colorbar(ax); ylabel(cb,'Depth (m)');
    
    % ----- Build segmented colormap -----
    % Normalize the threshold into [0,1] colormap domain
    k = max(1, min(N, round((th-near)/(far-near)*(N-1))+1));
    
    % Segment 1 (1..k): reds -> yellow/orange
    %   Red stays at 1; Green ramps 0 -> ~0.9; Blue stays 0
    seg1_R = ones(k,1);
    seg1_G = linspace(0,0.9,k).';
    seg1_B = zeros(k,1);
    seg1   = [seg1_R seg1_G seg1_B];
    
    % Segment 2 (k+1..N): start at yellow and move to cooler tones
    %   Use flipped parula so it begins yellow and fades toward blue
    par = parula(N); 
    seg2_full = flipud(par);              % starts near yellow
    seg2 = seg2_full(1:(N-k), :);         % keep exactly what's needed
    
    % Final map
    cmap = [seg1; seg2];
    colormap(ax, cmap);
    set(ax,'CLim',[near far]);            % lock limits so colorbar matches
        [v, u] = find(depth_img < 0.30) ;
        coll_ratio = length(u) / (640*480) ;
        qstr = sprintf('less than 30cm pxs: %5.2f %%',coll_ratio*100) ;
    title(qstr);

end