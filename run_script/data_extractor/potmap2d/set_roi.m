function [costmap_roi, angle_map_roi, roi_front_px, roi_back_px] = set_roi( costmap,  angle_map, cm_params) 

    resolution = cm_params.resolution ;
    map_size_m = cm_params.map_size_m ;

    % Get costmap ROI
    roi_front_m = 1.0 ; assert( roi_front_m < map_size_m ) ;
    roi_back_m = 0.0 ; assert( roi_back_m <= roi_front_m) ;

    roi_front_px = roi_front_m / resolution ; 
    roi_back_px = roi_back_m / resolution ;
    
    map_size_px = map_size_m / resolution ;
    rx = map_size_px / 2 ;  % robot cent
    ry = rx ;

    costmap_roi = costmap( rx - roi_back_px : rx + roi_front_px, ry - roi_front_px : ry + roi_front_px  ) ;
    angle_map_roi = angle_map( rx - roi_back_px : rx + roi_front_px, ry - roi_front_px : ry + roi_front_px  ) ;

    rx_roi = roi_back_px ;
    ry_roi = roi_front_px / 2 ;

end