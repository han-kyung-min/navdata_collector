#! /usr/bin/env python

import sys
import os
import pathlib
import yaml
import glob
import datetime
import cv2

import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import shutil
import rospy
import rosbag
import sensor_msgs.msg
from navdata_collector.msg import rgbd_metadata
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from tqdm import tqdm
from tqdm import trange

class scan_data_handler():
    def __init__(self, name, bagfile_path, **kwargs):
        self.name = name

        #rospy.Subscriber('scan_metadata', scan_metadata, self.scanMetadataCallBack)
        #self.header
        self.map    = nav_msgs.msg.OccupancyGrid
        self.depth  = sensor_msgs.msg.Image
        self.rgb    = sensor_msgs.msg.Image
        self.pose   = geometry_msgs.msg.Pose
        self.twist  = geometry_msgs.msg.Twist

        self.bagfile_path  = bagfile_path
        self.out_path      = kwargs['bagfile_decoder']['outpath']
        self.extraction_path    = str

        self.metadata_topic   = kwargs['navdata_collector']['metadata_topc']
        # self.map_topic      = kwargs['navdata_collector']['map_topic']
        # self.pose_topic     = kwargs['navdata_collector']['robotpose_topic']
        # self.twist_topic    = kwargs['navdata_collector']['twiststamped_topic']

        self.bagfile_idx    = 0
        self.bagfiles       = glob.glob('%s/*.bag' % self.root_path)
        self.bagfiles.sort()
        self.bag            = rosbag

        # create paths
        num_bagfiles = len(self.bagfiles)
        # now = datetime.datetime.now()
        # curr_time = now.strftime('%Y-%m-%d_%H-%M')

        # read first bag file to decode date time info
        bagfile0 = self.bagfiles[0]
        stridx = bagfile0.find('bag_')
        bagfile_time_str = bagfile0[stridx+4:-4]
        bag_time = bagfile_time_str.split('-')

        self.extraction_path = '%s/%s-%s_%s-%s'%(self.out_path, bag_time[0], bag_time[1], bag_time[2], bag_time[3] )
        print(self.extraction_path)
        if os.path.isdir( self.extraction_path ):
            shutil.rmtree( self.extraction_path )

        os.mkdir(self.extraction_path)

        self.out_depth_paths= []
        self.out_rgb_paths  = []
        self.out_pose_paths = []
       # self.out_map_paths  = []
        self.totnum_bagfiles = 0

        #print(self.bagfiles)
        # for bagfile_idx in range(0, len(self.bagfiles)):
        #     out_traj_path = '%s/traj%05d' % (self.extraction_path, bagfile_idx)
        #     os.mkdir(out_traj_path)
        #     out_scan_path = '%s/scan' % (out_traj_path)
        #     out_pose_path = '%s/pose' % (out_traj_path)
        #     out_map_path  = '%s/map'  % (out_traj_path)
        #     os.mkdir( out_scan_path )
        #     os.mkdir( out_pose_path )
        #     os.mkdir( out_map_path )
        #     self.out_scan_paths.append(out_scan_path)
        #     self.out_map_paths.append(out_map_path)
        #     self.out_pose_paths.append(out_pose_path)
        #     self.totnum_bagfiles = self.totnum_bagfiles + 1

    def runExtractor(self ):
        # for bagfile in bagfiles:
        #     # rosbag play each file then dump files into the dest folder
        bagfile = self.bagfiles[0]

        # open bag file to start writing
        bagfile_idx = 0
        pbar = tqdm(self.bagfiles)
        for bagfile in pbar:
            self.bag = rosbag.Bag(bagfile)
            #rospy.loginfo("processing the bag file: < %s > \n" % bagfile)
            pbar.set_description("Processing %s" % bagfile)

            frameidx = 0
            #for idx in range(0, self.totnum_bagfiles):

            out_traj_path = '%s/traj%05d' % (self.extraction_path, bagfile_idx)
            os.mkdir(out_traj_path)
            out_depth_path  = '%s/depth' % (out_traj_path)
            out_rgb_path    = '%s/rgb' % (out_traj_path)
            out_pose_path   = '%s/pose' % (out_traj_path)
            #out_map_path    = '%s/map' % (out_traj_path)
            os.mkdir(out_depth_path)
            os.mkdir(out_rgb_path)
            os.mkdir(out_pose_path)
            #os.mkdir(out_map_path)
            self.out_depth_paths.append(out_depth_path)
            self.out_rgb_paths.append(out_rgb_path)
            #self.out_map_paths.append(out_map_path)
            self.out_pose_paths.append(out_pose_path)
            self.totnum_bagfiles = self.totnum_bagfiles + 1

            frameidx = 0
            for topic, msg, t in self.bag.read_messages(topics=[self.metadata_topic]):
                #print(msg.scan.ranges)
                out_depth_file = '%s/depth%05d.txt' %(self.out_depth_paths[bagfile_idx], frameidx)
                out_rgb_file  = '%s/rgb%05d.png' %(self.out_rgb_paths[bagfile_idx], frameidx)
                #out_map_info_file = '%s/info%05d.txt' %(self.out_map_paths[bagfile_idx], frameidx)
                out_pose_file = '%s/pose%05d.txt' %(self.out_pose_paths[bagfile_idx], frameidx)
                out_twist_file= '%s/twist%05d.txt'%(self.out_pose_paths[bagfile_idx], frameidx)
                #print("openning scan file: < %s >" %(out_scan_file) )
                self.writeScanData(out_depth_file, msg.depth)
    #self.writeMapData (out_map_file, out_map_info_file, msg.map)
                self.writePoseData (out_pose_file, msg.rpose)
                frameidx += 1
            self.bag.close()
            bagfile_idx += 1


    # def scanMetadataCallBack(self, scan_data):
    #     self.header = scan_data.header
    #     self.map = scan_data.map
    #     self.scan = scan_data.scan
    #     self.vel = scan_data.cmd_vel
    #     self.rpose = scan_data.pose
    #     rospy.loginfo(rospy.get_caller_id() + ": I heard %s", self.header.stamp)
    #
    #     out_traj_path = '%s/traj%05d' % (self.out_path, self.bagfile_idx)
    #
    #     # scan message
    #     out_scan_file = '%s/scan/scan%05d.txt' % self.out_path
    #     self.writeScanData( self, out_scan_file, self.scan)

    def writeScanData(self, out_scan_file, scan_msg):
        data = scan_msg.ranges
        f = open(out_scan_file, "w")
        for idx in range(0, len(data)):
            f.write('%f '% data[idx])
        f.close()

    def writeMapData(self, out_img_file, out_info_file, map_msg):
        data = map_msg.data
        cols = map_msg.info.width
        rows = map_msg.info.height
        res  = map_msg.info.resolution
        ox   = map_msg.info.origin.position.x
        oy   = map_msg.info.origin.position.y
        img = np.zeros([rows, cols], dtype=np.uint8)
        for ridx in range(0, rows):
            for cidx in range(0, cols):
                val = data[ ridx * cols + cidx ]
                if val < 0:
                    img[ridx, cidx] = 127
                elif val > 50:
                    img[ridx, cidx] = 255
                else:
                    img[ridx, cidx] = 0
        cv2.imwrite( out_img_file, img)
        f = open( out_info_file, "w" )
        f.write('%d %d %f %f %f'% ( cols, rows, ox, oy, res) )
        #f = open(out_map_file)
    def writePoseData(self, out_pose_file, pose_msg):
        x   = pose_msg.position.x
        y   = pose_msg.position.y
        qx  = pose_msg.orientation.x
        qy  = pose_msg.orientation.y
        qz  = pose_msg.orientation.z
        qw  = pose_msg.orientation.w
        f   = open(out_pose_file, "w")
        f.write('%f %f %f %f %f %f %f' % (x, y, 0., qx, qy, qz, qw))
        f.close()

    def openScanDataPath(out_traj_path, bagfile):
        scandata_path = '%s/scan' % out_traj_path
        pose_path = '%s/pose' % out_traj_path
        mapdata_path = '%s/map' % out_traj_path
        if (os.path.isdir(scandata_path) == True):
            shutil.rmtree(out_traj_path)
        os.mkdir(scandata_path)
        os.mkdir(pose_path)
        os.mkdir(mapdata_path)
        bag = rosbag.Bag(bagfile)

    def closeScanDataPath(bag):
        bag.close()
