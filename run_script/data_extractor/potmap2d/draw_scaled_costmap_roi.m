function [] = draw_scaled_costmap_roi() 

win = 10;                 % half-size → 20x20 window
cx = round(gx);
cy = round(gy);

[H, W] = size(costmap_i8);

% --- 1) (gx,gy) 중심 20×20 crop 영역 계산 ---
x1 = max(1, cx - win);
x2 = min(W, cx + win - 1);
y1 = max(1, cy - win);
y2 = min(H, cy + win - 1);

window_20 = costmap_i8(y1:y2, x1:x2);

% --- 2) 4배 확대 (20x20 → 80x80) ---
scale_factor = 32;
vstep = 1 ;
zoom_win = imresize(window_20, scale_factor, 'nearest');

imshow(zoom_win, []);
colormap(gray);
axis on;
set(gca, 'YDir', 'normal');
hold on;
title(sprintf('Zoomed 20×20 @ (%.0f, %.0f), ×%d', gx, gy, scale_factor));

% ===============================
% 3) subgoal을 zoom 좌표계로 변환
% ===============================

% (1) 로봇 기준 subgoal → 전체 costmap 픽셀 좌표
sgs_sel = sgs_xyzq_corrected_px(cands_idx,:); 
sgs_sel(:,1:2) = sgs_sel(:,1:2) + rx ;

if ~isempty(sgs_sel)
    % (3) crop local 좌표 (window_20 기준) → zoom 좌표로 변환
    %   - crop 좌상단 (x1,y1)이 zoom 이미지의 (1,1)에 해당
    x_local = sgs_sel(:,1) - x1 + 1;   % 1 ~ 20
    y_local = sgs_sel(:,2) - y1 + 1;

    x_zoom = x_local * scale_factor;          % 1 ~ 80
    y_zoom = y_local * scale_factor;

    % drawSubGoalPoses는 x,y에 rx,ry를 더해서 그리므로,
    % 이미 zoom 좌표로 변환한 값만 넣고 rx=ry=0으로 호출하면 됨
    sgs_sel(:,1) = x_zoom;
    sgs_sel(:,2) = y_zoom;

    % ===============================
    % 4) drawSubGoalPoses로 subgoal 그리기
    % ===============================
    sgs_px_cands = [sgs_sel(:,1) +  rx, sgs_sel(:,2) +  ry] ;
    quat = sgs_sel(:,4:end) ;
    yaw = atan2(2*(quat(:,1).*quat(:,4) + quat(:,2).*quat(:,3)), 1 - 2.*(quat(:,3).^2 + quat(:,4).^2));
    
    dirvec = [cos(yaw), sin(yaw)];
    scale = scale_factor * 2 ;
    
    u = sgs_sel(1:vstep:end,1) ;
    v = sgs_sel(1:vstep:end,2) ;
    dvx = scale* dirvec(1:vstep:end,1);
    dvy = scale* dirvec(1:vstep:end,2);
    
    plot( u, v, 'bo', 'MarkerSize', 16, 'MarkerFaceColor', 'b' ) ;
    hold on;
    h = quiver(u, v, dvx, dvy, 0, 'LineWidth', 1, 'Color', 'r', 'MaxHeadSize', scale )
    h.MaxHeadSize = scale ;
end

% (옵션) 클릭한 점도 함께 표시
cx_local = (cx - x1 + 1) * scale_factor;
cy_local = (cy - y1 + 1) * scale_factor;
plot(cx_local, cy_local, 'gx', 'MarkerSize', 10, 'LineWidth', 2);

[gx, gy, button] = ginput(1) ;
if button == 3
    disp('skipping 10 frames \n');
else
    step_idx = step_idx + 1;
    if (gx <0 | gx > map_size_px | gy < 0 | gy > map_size_px)
        continue;  % if the not qualitifed to be a collision data. Don't bother
    end
end