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

class metadata_syncer():

    def __init__(self, name, **kwargs):
        self.name = name
        self.bagfile_path         = kwargs['navdata_extractor']['inpath']
        self.base_extraction_path = kwargs['navdata_extractor']['outpath']
        self.navtime_id           = self.bagfile_path.split('/')[-1]
        self.extracted_data_path      = '%s/%s'%(self.base_extraction_path, self.navtime_id)

        self.config_colldata_extractor = kwargs.get('colldata_extractor')

    def associate_time(self, ref_idx, ref_time, time_diff_thr, tgt_time):

        ref_tgt_matching_warn = 0
        tgt_time_all = tgt_time
        tgt_ref_time_diff = abs(tgt_time_all - ref_time)
        idx = np.where(tgt_ref_time_diff == np.min(tgt_ref_time_diff))[0]
        matching_tgt_idx = idx[0]
        #ref_tgt_time_diffs[ref_idx] = np.min(tgt_ref_time_diff)
        min_time_diff = np.min(tgt_ref_time_diff)

        if (min_time_diff > time_diff_thr):
            msg = "warning: @ rgb_idx <%d> min sg and rgb time diff is greator than %f (ms) \n" % (
            ref_idx, time_diff_thr * 1000)
            warnings.warn(msg)
            print('\033[33m' + msg + '\33[0m')
            ref_tgt_matching_warn += 1

        return matching_tgt_idx, min_time_diff, ref_tgt_matching_warn

    def match_rgbd_odom_colldata( self, rgb_info, depth_info, scan_info, odom, subgoal, waypoint, joy, my_time_diff = 0.1):
        # time_thr unit is < ms > i.e.  nsec / 10^6 )
        # reference is waypoint
        ref_data = rgb_info.copy()
        ref_size = len(ref_data)
        corr_table = np.zeros( [ref_size, 7], dtype=np.uint )  # rgb, depth, scan, odom, wp, sg, joy

        ref_depth_matching_warn = 0
        ref_scan_matching_warn = 0
        ref_odom_matching_warn = 0
        ref_wp_matching_warn = 0
        ref_subgoal_matching_warn = 0
        ref_joy_matching_warn = 0

        ref_depth_time_diffs = np.zeros( [ref_size,1] )
        ref_scan_time_diffs  = np.zeros( [ref_size,1] )
        ref_odom_time_diffs = np.zeros( [ref_size,1] )
        ref_wp_time_diffs   = np.zeros( [ref_size,1] )
        ref_subgoal_time_diffs = np.zeros( [ref_size,1] )
        ref_joy_time_diffs = np.zeros( [ref_size,1] )

        for ref_idx in range(0, ref_size):
            ref_s = ref_data[ref_idx, 4]
            ref_ns= ref_data[ref_idx, 5]
            ref_time = ref_s + ref_ns / 10 ** 9

            # find the assoicated depth data
            depth_time = depth_info[:, 4] + depth_info[:, 5] / 10 ** 9
            matching_depth_idx, min_time_diff, ref_depth_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, depth_time)
            ref_depth_time_diffs[ref_idx] = min_time_diff
            ref_depth_matching_warn += ref_depth_matching_warn_incr

            # find the associated scan data
            scan_time = scan_info[:, 2] + scan_info[:, 3] / 10 ** 9
            matching_scan_idx, min_time_diff, ref_scan_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, scan_time)
            ref_scan_time_diffs[ref_idx] = min_time_diff
            ref_scan_matching_warn += ref_scan_matching_warn_incr

            # find the associated odom data
            odom_time = odom[:, 2] + odom[:, 3] / 10 ** 9
            matching_odom_idx, min_time_diff, ref_odom_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, odom_time)
            ref_odom_time_diffs[ref_idx] = min_time_diff
            ref_odom_matching_warn += ref_odom_matching_warn_incr

            # find the assoicated wp data
            wp_time = waypoint[:, 2] + waypoint[:, 3] / 10 ** 9
            matching_wp_idx, min_time_diff, ref_wp_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, wp_time)
            ref_wp_time_diffs[ref_idx] = min_time_diff
            ref_wp_matching_warn += ref_wp_matching_warn_incr

            # find the associated subgoal data
            sg_time = subgoal[:, 2] + subgoal[:, 3] / 10 ** 9
            matching_sg_idx, min_time_diff, ref_subgoal_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, sg_time)
            ref_subgoal_time_diffs[ref_idx] = min_time_diff
            ref_subgoal_matching_warn += ref_subgoal_matching_warn_incr

            # find the associated joy data
            joy_time = joy[:, 2] + joy[:, 3] / 10 ** 9
            matching_joy_idx, min_time_diff, ref_joy_matching_warn_incr = self.associate_time(ref_idx, ref_time, my_time_diff, joy_time)
            ref_joy_time_diffs[ref_idx] = min_time_diff
            ref_joy_matching_warn += ref_joy_matching_warn_incr
            corr_table[ref_idx] = [ref_idx, matching_depth_idx, matching_scan_idx, matching_odom_idx, matching_wp_idx, matching_sg_idx, matching_joy_idx ]

        if( ref_odom_matching_warn > 0):
            msg = "warning: |ref_time - odom_time| > %f for %d times \n"% (my_time_diff * 1000, ref_odom_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        if( ref_depth_matching_warn > 0):
            msg = "warning: |ref_time - depth_time | > %f for %d times \n"% (my_time_diff * 1000, ref_depth_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        if( ref_scan_matching_warn > 0):
            msg = "warning: |ref_time - scan_time | > %f for %d times \n"% (my_time_diff * 1000, ref_scan_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        if( ref_subgoal_matching_warn > 0):
            msg = "warning: |ref_time - subgoal_time | > %f for %d times \n"% (my_time_diff * 1000, ref_subgoal_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        if( ref_wp_matching_warn > 0):
            msg = "warning: |ref_time - wp_time| > %f for %d times \n"% (my_time_diff * 1000, ref_wp_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        if( ref_joy_matching_warn > 0):
            msg = "warning: |ref_time - joy_time| > %f for %d times \n"% (my_time_diff * 1000, ref_joy_matching_warn)
            print('\033[33m' + msg + '\33[0m')

        print("******************************************************************************************************************* \n")
        print("ref & depth time diff report (ms):   min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_depth_time_diffs)*1000, np.max(ref_depth_time_diffs)*1000, np.mean(ref_depth_time_diffs)*1000 ) )
        print("ref & scan time diff report (ms):    min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_scan_time_diffs)*1000, np.max(ref_scan_time_diffs)*1000, np.mean(ref_scan_time_diffs)*1000 ) )
        print("ref & wp time diff report (ms):      min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_wp_time_diffs)*1000, np.max(ref_wp_time_diffs)*1000, np.mean(ref_wp_time_diffs)*1000 ) )
        print("ref & odom time diff report (ms):    min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_odom_time_diffs)*1000, np.max(ref_odom_time_diffs)*1000, np.mean(ref_odom_time_diffs)*1000 ) )
        print("ref & subgoal time diff report (ms): min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_subgoal_time_diffs)*1000, np.max(ref_subgoal_time_diffs)*1000, np.mean(ref_subgoal_time_diffs)*1000 ) )
        print("ref & joy time diff report (ms):     min: %.5f \t max: %.5f \t avg: %.5f \n"% (np.min(ref_joy_time_diffs)*1000, np.max(ref_joy_time_diffs)*1000, np.mean(ref_joy_time_diffs)*1000 ) )
        print("******************************************************************************************************************* \n")

        return corr_table

    def match_rgb_depth_odom( self, rgb_info, depth_info, odom, my_time_diff = 0.1):
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

    def match_rgb_depth_odom_twist( self, rgb_info, depth_info, odom, twist, time_diff_thr_s = 0.01):
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

    def match_rgb_scan_odom( self, rgb_info, scan_info, odom, my_time_diff = 0.1):
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

    def sync_metadata(self ):
        extraction_path      = '%s/%s'%(self.base_extraction_path, self.navtime_id)
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
            odom        = np.loadtxt( odom_file )
            twist       = np.loadtxt( twist_file )
            scan_info   = np.loadtxt( scan_info_file )

            if self.config_colldata_extractor:
                sg_info_file = '%s/rel_subgoals/rel_subgoal.txt'
                wp_info_file = '%s/waypoints/waypoints.txt'
                joy_info_file = '%s/joy/joy.txt'
                subgoal = np.loadtxt( '%s/rel_subgoals/rel_subgoal.txt'%rawdata_dir )
                waypoint = np.loadtxt( '%s/waypoints/waypoints.txt'%rawdata_dir )
                joy = np.loadtxt('%s/joy/joy.txt' % rawdata_dir)
                corr_table = self.match_rgbd_odom_colldata(rgb_info=rgb_info, depth_info=depth_info, scan_info=scan_info,
                                                           odom=odom, subgoal=subgoal, waypoint=waypoint, joy=joy )
                #corr_table = [wp_idx, matching_rgb_idx, matching_depth_idx, matching_odom_idx, matching_sg_idx, matching_joy_idx ]

                corr_table_file = '%s/rgbd_odom_colldata_matches.txt' % rawdata_dir
                np.savetxt(corr_table_file, corr_table, fmt='%d')
                print("time matched table is saved in %s\n"%corr_table_file )
            else:
                corr_table  = self.match_rgb_depth_odom(rgb_info=rgb_info, depth_info=depth_info, odom=odom, my_time_diff=0.1)
                # corr_table = [rgb_idx, matching_depth_idx, matching_odom_idx]
                corr_table_file = '%s/rgb_depth_odom_matches.txt' % rawdata_dir
                np.savetxt(corr_table_file, corr_table, fmt='%d')
                print("time matched table is saved in %s\n"%corr_table_file )
            
        print("corr table generation process took  %.2f seconds ---" % (time.time() - start_time))

    #################################### gen synced dataset ##########################################

        metadata_dirs        = glob.glob('%s/bag_*' % self.extracted_data_path)
        metadata_dirs.sort()

        start_time = time.time()
        for metadata_dir in metadata_dirs:
            print("=================================================================================================================\n")
            print("processing the raw metadata: %s \n" % metadata_dir)

            view_dir = '%s/synced' % metadata_dir
            if os.path.isdir(view_dir):
                shutil.rmtree(view_dir)
            os.mkdir(view_dir)

            # load rgb info to get img size()
            [h_rgb, w_rgb] = np.loadtxt('%s/rgb/rgb_info.txt'%metadata_dir)[0][1:3]
            [h_dep, w_dep] = np.loadtxt('%s/depth/depth_info.txt'%metadata_dir)[0][1:3]
            # load odom data
            odom = np.loadtxt('%s/traj/odom.txt' % metadata_dir)

            rgb_loc = None; depth_loc = None; scan_loc = None; odom_loc = None; wp_loc = None; sg_loc = None; joy_loc = None
            if self.config_colldata_extractor:
                corr_table_file = '%s/rgbd_odom_colldata_matches.txt' % metadata_dir
                rgb_loc = 0; depth_loc = 1; scan_loc=2; odom_loc = 3; wp_loc = 4;  sg_loc = 5; joy_loc = 6
                # corr_table = [rgb_idx, matching_depth_idx, matching_odom_idx, matchin_wp_idx, matching_sg_idx, matching_joy_idx ]
            else:
                corr_table_file = '%s/rgb_depth_odom_matches.txt' % metadata_dir
                rgb_loc = 0; depth_loc = 1; odom_loc = 2;
                # corr_table = [rgb_idx, matching_depth_idx, matching_odom_idx]
            corr_table = np.loadtxt(corr_table_file).astype('uint64')

            # sync odom first
            sync_odom_file = '%s/synced/sync_odom.txt' % metadata_dir
            sync_odom_arr = np.zeros([len(corr_table), odom.shape[1]])

            # process odom sync
            for ii in range(0, len(corr_table)):
                sync_odom_line = odom[corr_table[ii][odom_loc]]
                sync_odom_arr[ii] = sync_odom_line
                np.savetxt(sync_odom_file, sync_odom_arr)

            # process colldata sync if necesssary
            # corr_table = [wp_idx, matching_depth_idx, matching_odom_idx, matching_sg_idx, matching_joy_idx]
            if self.config_colldata_extractor:
                # corr_table = [wp_idx, matching_rgb_idx, matching_depth_idx, matching_scan_idx, matching_odom_idx, matching_sg_idx, matching_joy_idx ]
                subgoal = np.loadtxt('%s/rel_subgoals/rel_subgoal.txt' % metadata_dir)
                sync_sg_file = '%s/synced/sync_rel_subgoals.txt' % metadata_dir
                sync_sg_arr = np.zeros([len(corr_table), subgoal.shape[1]])
                for ii in range(0, len(corr_table)):
                    sync_sg_line = subgoal[corr_table[ii][sg_loc]]
                    sync_sg_arr[ii] = sync_sg_line
                np.savetxt(sync_sg_file, sync_sg_arr)


                waypoint = np.loadtxt('%s/waypoints/waypoints.txt' % metadata_dir)
                sync_wp_file = '%s/synced/sync_waypoints.txt' % metadata_dir
                sync_wp_arr = np.zeros([len(corr_table), waypoint.shape[1]])
                for ii in range(0, len(corr_table)):
                    sync_wp_line = waypoint[corr_table[ii][wp_loc]]
                    sync_wp_arr[ii] = sync_wp_line
                np.savetxt(sync_wp_file, sync_wp_arr)

                joy = np.loadtxt('%s/joy/joy.txt' % metadata_dir)
                sync_joy_file = '%s/synced/sync_joy.txt' % metadata_dir
                sync_joy_arr = np.zeros([len(corr_table), joy.shape[1]])
                for ii in range(0, len(corr_table)):
                    sync_joy_line = joy[corr_table[ii][joy_loc]]
                    sync_joy_arr[ii] = sync_joy_line
                np.savetxt(sync_joy_file, sync_joy_arr)

                # scan data
                for ii in range(0, len(corr_table)):
                    print('saving synced %d th scan' % ii, end='\r')
                    src_scan_file = '%s/scan/%05d.txt' % (metadata_dir, corr_table[ii][scan_loc])
                    dst_sync_scan_file = '%s/synced/scan%05d.txt' % (metadata_dir, ii)
                    shutil.copyfile(src_scan_file, dst_sync_scan_file)


            for ii in range(0, len(corr_table)) :
                print('saving synced %d th metadata' % ii, end='\r')
                src_rgb_file    = '%s/rgb/%05d.png' % (metadata_dir, corr_table[ii][rgb_loc])
                src_depth_file  = '%s/depth/%05d.png' % (metadata_dir, corr_table[ii][depth_loc])

                dst_sync_rgb_file = '%s/synced/rgb%05d.png' % (metadata_dir, ii)
                dst_sync_depth_file = '%s/synced/depth%05d.png' % (metadata_dir, ii)
                shutil.copyfile(src_rgb_file, dst_sync_rgb_file)
                shutil.copyfile(src_depth_file, dst_sync_depth_file)





        print("meta-data sync process took  %.2f seconds ---" % (time.time() - start_time))
        
