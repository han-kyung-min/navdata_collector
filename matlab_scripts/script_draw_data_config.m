% draw data configuration


root_data_folder = '/media/mydata/former_datasets/colldata/colldata_all';

for idx=100:400

    data_folder = sprintf('%s/data%05d',root_data_folder, idx) ;
    wp_file = sprintf('%s/corrected_waypoints_m.txt', data_folder) ;
    %context_file = 

    wps_m = load(wp_file) ;

    figure(1); clf; hold on;
    plot(wps_m(1,1), wps_m(1,2), 'ro') ; hold on;
    plot(wps_m(2,1), wps_m(2,2), 'g*') ; hold on;
    plot(wps_m(3,1), wps_m(3,2), 'b<') ; hold on;
    plot(wps_m(4,1), wps_m(4,2), 'm>') ; hold on;
    plot(wps_m(5,1), wps_m(5,2), 'cs') ; 

    pause

    close all;
end