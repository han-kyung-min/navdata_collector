#! /usr/bin/env python
import sys
import os
import pathlib
import yaml
import glob
import random
import time
import numpy as np
import rospy
import cv2
import shutil

def world2gridmap( wx, wy, ox, oy, res):
    fx = (wx - ox) / res
    fy = (wy - oy) / res
    return [ int(fx), int(fy)]

def main(argv):

    if(len(sys.argv) != 2):
        print( "usage: %s <decoded_traj_paths>" % sys.argv[0] )
        return -1

    root_path = sys.argv[1]
    if( os.path.isdir( root_path ) is False ):
        print("There is no  < %s > folder. Double check the input dir \n"% sys.argv[1] )

    trajs = os.listdir(root_path)
    root_dir = sys.argv[1]
    # count traj folders

    debug_view_path = '%s/robot_on_maps' % (root_dir)
    print(debug_view_path)
    if os.path.isdir(debug_view_path):
        shutil.rmtree(debug_view_path)
    os.mkdir(debug_view_path)

    # viewing
    traj_idx = 0
    for traj in trajs:
        map_img_path    = '%s/%s/map'  % (root_dir, traj)
        scan_path       = '%s/%s/scan' % (root_dir, traj)
        pose_path       = '%s/%s/pose' % (root_dir, traj)

        mapimg_files    = glob.glob('%s/*.png' % map_img_path)
        mapinfo_files   = glob.glob('%s/*.txt' % map_img_path)
        pose_files      = glob.glob('%s/*.txt' % pose_path)
        mapimg_files.sort()
        mapinfo_files.sort()
        pose_files.sort()

        fidx = 0
        for mapimg_file, mapinfo_file, pose_file in zip(mapimg_files, mapinfo_files, pose_files):
            mapimg = cv2.imread(mapimg_file)
            fid = open(mapinfo_file, 'r')
            info = fid.readlines()
            fid.close()
            s = info[0].split(' ')
            width = int( s[0] )
            height= int( s[1] )
            ox    = float( s[2] )
            oy    = float( s[3] )
            res   = float( s[4] )

            fid = open(pose_file, 'r')
            poseline = fid.readlines()
            fid.close()
            s = poseline[0].split(' ')
            rx_w  = float( s[0] )
            ry_w  = float( s[1] )

            [rx_g, ry_g] = world2gridmap(rx_w, ry_w, ox, oy, res)
            cv2.circle(mapimg, (rx_g, ry_g), 4, (255, 255, 0), -1, cv2.LINE_8, 0)
            outfile = '%s/traj_%05d_%05d.png'%(debug_view_path, traj_idx, fidx)
            cv2.imwrite(outfile, mapimg)
            fidx += 1
        traj_idx += 1

if __name__ == "__main__":
    main(sys.argv)