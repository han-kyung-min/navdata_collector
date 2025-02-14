
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
    extracted_data_path      = '%s/%s'%(base_extraction_path, navtime_id)
    sync_metadata_dirs        = glob.glob('%s/bag_*/synced' % extracted_data_path)
    sync_metadata_dirs.sort()
    start_time = time.time()

    start_time = time.time()
    for sync_metadata_dir in sync_metadata_dirs:
        print("=================================================================================================================\n")
        print("processing the synced metadata: %s \n" % sync_metadata_dir)

        # load rgb info to get img size()
        # load odom data
        odom = np.loadtxt('%s/sync_odom.txt' % sync_metadata_dir)
        num_data = len(odom)
        for ii in range(0, num_data) :
            rgb_file    = '%s/rgb%05d.png' % (sync_metadata_dir, ii)
            depth_file  = '%s/depth%05d.png' % (sync_metadata_dir, ii)
            sync_rgb = cv2.imread(rgb_file)
            sync_depth = cv2.imread(depth_file)

            sync_rgb_file = '%s/synced/rgb%05d.png' % (sync_metadata_dir, ii)
            sync_depth_file = '%s/synced/depth%05d.png' % (sync_metadata_dir, ii)
            #img = np.hstack( (rgb, depth) )
            #out_fig_file = '%s/matched-rgb-d%04d.png' % (view_dir, ii )
            cv2.imwrite( sync_rgb_file, sync_rgb)
            cv2.imwrite( sync_depth_file, sync_depth)

    print("meta-data sync process took  %.2f seconds ---" % (time.time() - start_time))

if __name__ == "__main__":
    main(sys.argv)