#! /usr/bin/env python
import sys
import os
import pathlib
import yaml
import glob
import random
import time
import numpy as np
import warnings
import shutil
import cv2

def match_rgb_depth_odom( rgb_info, depth_info, odom, my_time_diff = 0.1):
    # time_thr unit is < ms > i.e.  nsec / 10^6 )
    rgb_size = len(rgb_info)
    corr_table = np.zeros( [rgb_size, 3], dtype=np.uint )  # rgb, depth, odom
    rgb_depth_matching_warn = 0
    rgb_odom_matching_warn = 0
    rgbd_time_diffs = np.zeros( [rgb_size,1] )
    rgb_odom_time_diffs = np.zeros( [rgb_size,1] )

    for rgb_idx in range(0, rgb_size):
        rgb_s = rgb_info[rgb_idx, 4]
        rgb_ns= rgb_info[rgb_idx, 5]
        rgb_time = rgb_s + rgb_ns / 10 ** 9

        # find the assoicated depth data
        depth_secs = depth_info[:,4]
        depth_nsecs= depth_info[:,5] / 10 ** 9
        depth_time_all = depth_secs + depth_nsecs

        depth_rgb_time_diff = abs( depth_time_all - rgb_time )
        idx = np.where( depth_rgb_time_diff == np.min(depth_rgb_time_diff) )[0]
        matching_depth_idx = idx[0]
        rgbd_time_diffs[rgb_idx] = np.min(depth_rgb_time_diff)

        if( np.min(depth_rgb_time_diff) > my_time_diff ):
            msg = "warning: @ rgb_idx <%d> min depth and rgb time diff is greator than %f (ms) \n"% (rgb_idx, my_time_diff * 1000)
            print('\033[33m' + msg + '\33[0m')
            rgb_depth_matching_warn += 1

        # find the associated odom data
        odom_secs = odom[:,2]
        odom_nsecs = odom[:,3] / 10 ** 9
        odom_time_all = odom_secs + odom_nsecs
        odom_rgb_time_diff = abs( odom_time_all - rgb_time )
        idx = np.where( odom_rgb_time_diff == np.min(odom_rgb_time_diff) )[0]
        matching_odom_idx = idx[0]
        rgb_odom_time_diffs[rgb_idx] = np.min(odom_rgb_time_diff)

        if( np.min(odom_rgb_time_diff) > my_time_diff ):
            msg = "warning: @ rgb_idx <%d> min odom and rgb time diff is greator than %f (ms) \n" % (rgb_idx, my_time_diff * 1000)
            warnings.warn( msg )
            print('\033[33m' + msg + '\33[0m')
            rgb_odom_matching_warn += 1

        corr_table[rgb_idx] = [rgb_idx, matching_depth_idx, matching_odom_idx]

    if( rgb_odom_matching_warn > 0):
        msg = "warning: |odom_time - rgb_time| > %f for %d times \n"% (my_time_diff * 1000, rgb_odom_matching_warn)
        print('\033[33m' + msg + '\33[0m')

    if( rgb_depth_matching_warn > 0):
        msg = "warning: |depth_time - rgb_time| > %f for %d times \n"% (my_time_diff * 1000, rgb_depth_matching_warn)
        print('\033[33m' + msg + '\33[0m')
    print("******************************************************************************************************************* \n")
    print("rgb & depth time diff report (ms):  min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgbd_time_diffs)*1000, np.max(rgbd_time_diffs)*1000, np.mean(rgbd_time_diffs)*1000 ) )
    print("rgb & odom time diff report (ms):   min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgb_odom_time_diffs)*1000, np.max(rgb_odom_time_diffs)*1000, np.mean(rgb_odom_time_diffs)*1000 ) )
    print("******************************************************************************************************************* \n")
    return corr_table


def match_rgb_depth_odom_twist( rgb_info, depth_info, odom, twist, time_diff_thr_s = 0.01):
    # time_thr unit is < ms > i.e.  nsec / 10^6 )
    rgb_size = len(rgb_info)
    corr_table = np.zeros( [rgb_size, 4], dtype=np.int64 )  # rgb, depth, odom, twist
    rgb_depth_matching_warn = 0
    rgb_odom_matching_warn = 0
    rgbd_time_diffs = np.zeros( [rgb_size, 1] )
    rgb_odom_time_diffs = np.zeros( [rgb_size, 1] )
    rgb_twist_time_diffs = np.zeros( [rgb_size, 1] )

    for rgb_idx in range(0, rgb_size):
        rgb_s = rgb_info[rgb_idx, 4]
        rgb_ns= rgb_info[rgb_idx, 5]
        rgb_time = rgb_s + rgb_ns / 10 ** 9

        # find the assoicated depth data
        depth_secs = depth_info[:, 4]
        depth_nsecs= depth_info[:, 5] / 10 ** 9
        depth_time_all = depth_secs + depth_nsecs

        depth_rgb_time_diff = abs( depth_time_all - rgb_time )
        idx = np.where( depth_rgb_time_diff == np.min(depth_rgb_time_diff) )[0]
        matching_depth_idx = idx[0]
        rgbd_time_diffs[rgb_idx] = np.min(depth_rgb_time_diff)

        if( np.min(depth_rgb_time_diff) > time_diff_thr_s ): # 10 ms
            msg = "warning: @ rgb_idx <%d> min depth and rgb time diff is greator than %f (ms) \n"% (rgb_idx, time_diff_thr_s * 1000)
            print('\033[33m' + msg + '\33[0m')
            rgb_depth_matching_warn += 1

        # find the associated odom data
        odom_secs = odom[:,2]
        odom_nsecs = odom[:,3] / 10 ** 9
        odom_time_all = odom_secs + odom_nsecs
        odom_rgb_time_diff = abs( odom_time_all - rgb_time )
        idx = np.where( odom_rgb_time_diff == np.min(odom_rgb_time_diff) )[0]
        matching_odom_idx = idx[0]
        rgb_odom_time_diffs[rgb_idx] = np.min(odom_rgb_time_diff)

        # find the associated twist data
        twist_secs = twist[:, 2]
        twist_nsecs = twist[:, 3] / 10 ** 9
        twist_time_all = twist_secs + twist_nsecs
        twist_rgb_time_diff_s = abs( twist_time_all - rgb_time )
        idx = np.where( twist_rgb_time_diff_s == np.min( twist_rgb_time_diff_s ) )[0]
        matching_twist_idx = idx[0]
        rgb_twist_time_diffs[rgb_idx] = np.min(twist_rgb_time_diff_s)

        if( np.min(odom_rgb_time_diff) > time_diff_thr_s ):
            msg = "warning: @ rgb_idx <%d> min odom and rgb time diff is greator than %f (ms) \n" % (rgb_idx, time_diff_thr_s * 1000)
            warnings.warn( msg )
            print('\033[33m' + msg + '\33[0m')
            rgb_odom_matching_warn += 1

        if( np.min(twist_rgb_time_diff_s) > time_diff_thr_s):
            corr_table[rgb_idx] = [rgb_idx, matching_depth_idx, matching_odom_idx, -1]
        else:
            corr_table[rgb_idx] = [rgb_idx, matching_depth_idx, matching_odom_idx, matching_twist_idx]

    if( rgb_odom_matching_warn > 0):
        msg = "warning: |odom_time - rgb_time| > %f for %d times \n"% (time_diff_thr_s * 1000, rgb_odom_matching_warn)
        print('\033[33m' + msg + '\33[0m')

    if( rgb_depth_matching_warn > 0):
        msg = "warning: |depth_time - rgb_time| > %f for %d times \n"% (time_diff_thr_s * 1000, rgb_depth_matching_warn)
        print('\033[33m' + msg + '\33[0m')
    print("******************************************************************************************************************* \n")
    print("rgb & depth time diff report (ms):  min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgbd_time_diffs)*1000, np.max(rgbd_time_diffs)*1000, np.mean(rgbd_time_diffs)*1000 ) )
    print("rgb & odom time diff report (ms):   min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgb_odom_time_diffs)*1000, np.max(rgb_odom_time_diffs)*1000, np.mean(rgb_odom_time_diffs)*1000 ) )
    print("******************************************************************************************************************* \n")
    return corr_table


def match_rgb_scan_odom( rgb_info, scan_info, odom, my_time_diff = 0.1):
    # my_time_diff unit is < sec > )

    rgb_size = len(rgb_info)
    corr_table = np.zeros( [rgb_size, 3], dtype=np.uint )  # rgb, depth, odom
    rgb_scan_matching_warn = 0
    rgb_odom_matching_warn = 0

    rgb_info_size   = len(rgb_info)
    scan_info_size = len(scan_info)
    assert(scan_info_size > rgb_info_size)  # rgb and depth recorded lower rate than lidar owing to the memory problem

    rgb_scan_dt = np.zeros([rgb_size,1])
    rgb_odom_dt = np.zeros([rgb_size,1])

    for rgb_idx in range(0, rgb_size):
        rgb_s = rgb_info[rgb_idx,4]
        rgb_ns= rgb_info[rgb_idx,5]
        rgb_time = rgb_s + rgb_ns / 10 ** 9

        # find the assoicated scan data
        scan_secs = scan_info[:,7]
        scan_nsecs= scan_info[:,8] / 10 ** 9
        scan_time_all = scan_secs + scan_nsecs

        scan_rgb_time_diff = abs( scan_time_all - rgb_time )
        idx = np.where( scan_rgb_time_diff == np.min( scan_rgb_time_diff) )[0]
        matching_depth_idx = idx[0]
        rgb_scan_dt[rgb_idx] = np.min( scan_rgb_time_diff)

        if( np.min( scan_rgb_time_diff) > my_time_diff ):
            msg = "warning: @ rgb_idx <%d> min scan and rgb time diff is greator than %f (ms) \n"% (rgb_idx, my_time_diff * 1000)
            print('\033[33m' + msg + '\33[0m')
            rgb_scan_matching_warn += 1

        # find the associated odom data
        odom_secs = odom[:,2]
        odom_nsecs = odom[:,3] / 10 ** 9
        odom_time_all = odom_secs + odom_nsecs
        odom_rgb_time_diff = abs( odom_time_all - rgb_time )
        idx = np.where( odom_rgb_time_diff == np.min(odom_rgb_time_diff) )[0]
        matching_odom_idx = idx[0]
        rgb_odom_dt[rgb_idx] = np.min( odom_rgb_time_diff )

        if( np.min(odom_rgb_time_diff) > my_time_diff ):
            msg = "warning: @ rgb_idx <%d> min odom and rgb time diff is greator than %f (ms) \n" % (rgb_idx, my_time_diff * 1000)
            warnings.warn( msg )
            print('\033[33m' + msg + '\33[0m')
            rgb_odom_matching_warn += 1

        corr_table[rgb_idx] = [rgb_idx, matching_depth_idx, matching_odom_idx]

    if( rgb_odom_matching_warn > 0):
        msg = "warning: |odom_time - rgb_time| > %f for %d times \n"% (my_time_diff * 1000, rgb_odom_matching_warn)
        print('\033[33m' + msg + '\33[0m')

    if( rgb_scan_matching_warn > 0):
        msg = "warning: |scan_time - rgb_time| > %f for %d times \n"% (my_time_diff * 1000, rgb_scan_matching_warn)
        print('\033[33m' + msg + '\33[0m')

    print("******************************************************************************************************************* \n")
    print("rgb & scan time diff report (ms):  min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgb_scan_dt)*1000, np.max(rgb_scan_dt)*1000, np.mean(rgb_scan_dt)*1000 ) )
    print("rgb & odom time diff report (ms):  min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(rgb_odom_dt)*1000, np.max(rgb_odom_dt)*1000, np.mean(rgb_odom_dt)*1000 ) )
    print("******************************************************************************************************************* \n")
    return corr_table


def main(argv):
    """ This script goes over target folder (processed bag file) to associate
        rgb, depth, scan, and traj related metadata based on their time stamps.
        It will find the closest matching pairs, then write the table of matching indexes
     """
    if(len(sys.argv) != 2):
        print( "usage: %s <config_file>" % sys.argv[0] )
        return -1
    config_file = sys.argv[1]

    # config_file = '%s/param/navdata_collector.yaml'%proj_dir
    #config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    bagfile_path         = config['navdata_extractor']['inpath']
    base_extraction_path = config['navdata_extractor']['outpath']
    navtime_id           = bagfile_path.split('/')[-1]
    extraction_path      = '%s/%s'%(base_extraction_path, navtime_id)

    rawdata_dirs = glob.glob('%s/*/' % extraction_path)
    # rmidx = rawdata_dirs.index("%s/readme.txt"%base_extraction_path)
    # rawdata_dirs.pop(rmidx)
    rawdata_dirs.sort()
    # data association based on the time diff

    start_time = time.time()
    for rawdata_dir in rawdata_dirs:
        print("processing: %s \n"% rawdata_dir)
        rgb_info_file   = "%s/rgb/rgb_info.txt" % (rawdata_dir)
        depth_info_file = "%s/depth/depth_info.txt" % (rawdata_dir)
        scan_info_file = "%s/scan/scan_info.txt" % (rawdata_dir)
        odom_file = "%s/traj/odom.txt" % (rawdata_dir)
        twist_file = "%s/traj/twist.txt" % (rawdata_dir)
        # load rgb_info
        rgb_info    = np.loadtxt( rgb_info_file )
        depth_info  = np.loadtxt( depth_info_file )
        scan_info   = np.loadtxt( scan_info_file )
        odom        = np.loadtxt( odom_file )
        twist       = np.loadtxt( twist_file )
        corr_table  = match_rgb_depth_odom(rgb_info, depth_info, odom, my_time_diff=0.1)
        corr_table_file = '%s/rgb_depth_odom_matches.txt' % rawdata_dir
        np.savetxt(corr_table_file, corr_table, fmt='%d')
        print("time matched table is saved in %s\n"%corr_table_file )
    print("corr table generation process took  %.2f seconds ---" % (time.time() - start_time))
#################################### gen synced dataset ##########################################

    bagfile_path         = config['navdata_extractor']['inpath']
    base_extraction_path = config['navdata_extractor']['outpath']
    navtime_id           = bagfile_path.split('/')[-1]
    extracted_data_path      = '%s/%s'%(base_extraction_path, navtime_id)
    metadata_dirs        = glob.glob('%s/bag_*' % extracted_data_path)
    metadata_dirs.sort()

    start_time = time.time()
    for metadata_dir in metadata_dirs:
        print("=================================================================================================================\n")
        print("processing the raw metadata: %s \n" % metadata_dir)
        corr_table_file = '%s/rgb_depth_odom_matches.txt' % metadata_dir
        corr_table = np.loadtxt(corr_table_file).astype('uint64')

        view_dir = '%s/synced' % metadata_dir
        if os.path.isdir(view_dir):
            shutil.rmtree(view_dir)
        os.mkdir(view_dir)

        # load rgb info to get img size()
        [h_rgb, w_rgb] = np.loadtxt('%s/rgb/rgb_info.txt'%metadata_dir)[0][1:3]
        [h_dep, w_dep] = np.loadtxt('%s/depth/depth_info.txt'%metadata_dir)[0][1:3]
        # load odom data
        odom = np.loadtxt('%s/traj/odom.txt' % metadata_dir)

        # sync odom first
        sync_odom_file = '%s/synced/sync_odom.txt'%metadata_dir
        sync_odom_arr = np.zeros([len(corr_table), odom.shape[1] ])
        # process odom sync
        for ii in range(0, len(corr_table)):
            sync_odom_line = odom[corr_table[ii][2]]
            sync_odom_arr[ii] = sync_odom_line
        np.savetxt(sync_odom_file, sync_odom_arr)

        for ii in range(0, len(corr_table)) :
            print('saving synced %d th metadata' % ii, end='\r')
            src_rgb_file    = '%s/rgb/%05d.png' % (metadata_dir, corr_table[ii][0])
            src_depth_file  = '%s/depth/%05d.png' % (metadata_dir, corr_table[ii][1])

            dst_sync_rgb_file = '%s/synced/rgb%05d.png' % (metadata_dir, ii)
            dst_sync_depth_file = '%s/synced/depth%05d.png' % (metadata_dir, ii)
            #img = np.hstack( (rgb, depth) )
            #out_fig_file = '%s/matched-rgb-d%04d.png' % (view_dir, ii )
            #sync_rgb = cv2.imread(rgb_file)
            #sync_depth = cv2.imread(depth_file)
            #cv2.imwrite( sync_rgb_file, sync_rgb)
            #cv2.imwrite( sync_depth_file, sync_depth)
            shutil.copyfile(src_rgb_file, dst_sync_rgb_file)
            shutil.copyfile(src_depth_file, dst_sync_depth_file)
            

    print("meta-data sync process took  %.2f seconds ---" % (time.time() - start_time))

if __name__ == "__main__":
    main(sys.argv)
