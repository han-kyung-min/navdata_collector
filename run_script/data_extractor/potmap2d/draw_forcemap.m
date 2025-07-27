
function draw_forcemap(Urep, angle_map, map_size)
    
 %figure('Name', 'Potential and Orientation', 'NumberTitle', 'off');
%    t = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    %% --- Left Tile: Potential Magnitude ---
    %nexttile(t, 1);
    figure(1)
    imagesc(Urep);
    axis image;
    set(gca, 'YDir', 'normal');  % North up
    hcb = colorbar;
    ylabel(hcb,'Cost intensity','FontSize',16,'Rotation',270)
    %title('Repulsive Potential U_{rep}(q)');
    title('Costmap');
    xlabel('X (grid)');
    ylabel('Y (grid)');

    %% --- Middle Tile: Force Orientation Map ---

    %nexttile(t, 2);
    
    figure(2)
    %t = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    %nexttile(t, [1 2]);
    imagesc(angle_map);
    c = jet(360) ;
    colormap(c) ;
    colorbar
    axis image;
    set(gca, 'YDir', 'normal');
    title('Force Orientation (deg)', 'FontSize', 12);
    xlabel('X (grid)');
    ylabel('Y (grid)');

    % %% --- Right Tile: Color Circle Legend ---
    % ax_circle = nexttile(t, 3);
    % draw_orientation_circle(ax_circle);
end

function draw_orientation_circle(ax)
    axes(ax);
    cla(ax);
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');

    num_steps = 360;
    theta = linspace(0, 2*pi, num_steps) + pi/2;  % Rotate for north up
    radius = 40;
    cx = 50; cy = 50;

    % Draw colored circle with uniform arc segments
    for k = 1:num_steps-1
        hue = mod(theta(k) / (2*pi), 1);
        color = hsv2rgb([hue, 1, 1]);
    
        % Create a small arc with 2 points on the circle edge
        x_patch = [0, radius*cos(theta(k)), radius*cos(theta(k+1))];
        y_patch = [0, radius*sin(theta(k)), radius*sin(theta(k+1))];
    
        patch(cx + x_patch, cy + y_patch, color, 'EdgeColor', 'none', 'Parent', ax);
    end

    % Add direction arrows
    num_arrows = 8;
    arrow_angles = linspace(0, 2*pi, num_arrows + 1) + pi/2;
    for k = 1:num_arrows
        quiver(cx, cy, ...
               radius * cos(arrow_angles(k)), ...
               radius * sin(arrow_angles(k)), ...
               0, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.5, 'Parent', ax);
    end

    % Compass labels
    text(cx, cy - radius - 5, 'S', 'Color', 'k', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'Parent', ax);
    text(cx + radius + 5, cy, 'E', 'Color', 'k', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'Parent', ax);
    text(cx, cy + radius + 15, 'N', 'Color', 'k', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'Parent', ax);
    text(cx - radius - 10, cy, 'W', 'Color', 'k', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'Parent', ax);

    % Place "Orientation Legend" above N
    text(cx, cy + radius + 30, 'Orientation Legend', ...
         'Color', 'k', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'FontSize', 12, 'Parent', ax);

    hold(ax, 'off');
end




    % 
    % figure(1); clf; hold on;
    % subplot(1,2,1) ;
    % imagesc(Urep) ; %xlim([0, map_size]); ylim([0, map_size]); colorbar;
    % axis image;        % Keep aspect ratio
    % 
    % angle_map = rad2deg( atan2(Fry, Frx) ) ;
    % 
    % subplot(1,2,2);
    % imagesc(angle_map); %xlim([0, map_size]); ylim([0, map_size]);
    % axis image;        % Keep aspect ratio
    % %colorbar;
    % title('Force Orientation (Angle)');
    % hold on;
    % 
    % % Parameters for orientation circle
    % num_steps = 360;
    % theta = linspace(0, 2*pi, num_steps);
    % radius = 20;
    % cx = 40; cy = 40;
    % 
    % % Draw filled colored circle
    % for k = 1:num_steps-1
    %     hue = mod(theta(k) / (2*pi), 1);
    %     color = hsv2rgb([hue, 1, 1]);
    %     patch(cx + [0 radius*cos(theta(k)) radius*cos(theta(k+1))], ...
    %           cy + [0 radius*sin(theta(k)) radius*sin(theta(k+1))], ...
    %           color, 'EdgeColor', 'none');
    % end
    % 
    % % Add compass labels
    % text(cx, cy - radius - 5, 'N', 'Color', 'k', 'FontWeight', 'bold', ...
    %      'HorizontalAlignment', 'center');
    % text(cx + radius + 5, cy, 'E', 'Color', 'k', 'FontWeight', 'bold', ...
    %      'HorizontalAlignment', 'center');
    % text(cx, cy + radius + 15, 'S', 'Color', 'k', 'FontWeight', 'bold', ...
    %      'HorizontalAlignment', 'center');
    % text(cx - radius - 10, cy, 'W', 'Color', 'k', 'FontWeight', 'bold', ...
    %      'HorizontalAlignment', 'center');
    % 
    % % Draw vector arrows
    % num_arrows = 8;
    % arrow_angles = linspace(0, 2*pi, num_arrows + 1);
    % 
    % for k = 1:num_arrows
    %     x_start = cx;
    %     y_start = cy;
    %     x_end = cx + radius * cos(arrow_angles(k));
    %     y_end = cy + radius * sin(arrow_angles(k));
    %     quiver(x_start, y_start, ...
    %            x_end - x_start, y_end - y_start, ...
    %            0, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.5);
    % end


%end