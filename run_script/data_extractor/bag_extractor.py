#! /usr/bin/env python

import sys
import os
import pathlib
import yaml
import glob
import datetime
import cv2
from cv_bridge import CvBridge, CvBridgeError

import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import shutil
import rospy
import rosbag
import sensor_msgs.msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from tqdm import tqdm
from tqdm import trange

class bag_extractor():
    def __init__(self, name, **kwargs):
        self.name = name
        #self.rgb_topic      = kwargs['navdata_extractor']['rgb_topic']
        #self.depth_topic    = kwargs['navdata_extractor']['depth_topic']
        self.scan_topic     = kwargs['navdata_extractor']['scan_topic']
        self.odom_topic     = kwargs['navdata_extractor']['odom_topic']
        self.odom_filt_topic= kwargs['navdata_extractor']['odom_filt_topic']
        self.slam_pose_topic= kwargs['navdata_collector']['robotpose_topic']
        self.rgbd_topic = None
        self.rgb_topic = None
        self.depth_topic = None
                
        self.config_extractor = kwargs.get('navdata_extractor')
        if isinstance(self.config_extractor, dict):
            if 'rgbd_topic' in self.config_extractor:
                self.rgbd_topic     = kwargs['navdata_extractor']['rgbd_topic']
                print("we need to extract rgbd_topic: %s"%self.rgbd_topic)
            elif 'rgb_topic' in self.config_extractor:
                self.rgb_topic      = kwargs['navdata_extractor']['rgb_topic']
                self.depth_topic    = kwargs['navdata_extractor']['depth_topic']
                print("It is recommended to process RGB-D (combined) topic \n")
                raise NotImplementedError
            else:
                print("unknown image type\n")
                raise NotImplementedError
        
        self.twiststamped_topic    = kwargs['navdata_extractor']['twiststamped_topic']

        self.bagfile_path   = kwargs['navdata_extractor']['inpath']  #bagfile_path   # source dir
        self.base_extraction_path   = kwargs['navdata_extractor']['outpath']
        self.navtime_id     = self.bagfile_path.split('/')[-1]

        # fix the inactive bag if there is any.
        active_bags = glob.glob('%s/*.bag.active' % self.bagfile_path)
        for active_bag in active_bags:
            print("\n fixing %s to a normal bag file \n"%active_bag)
            targ_ = active_bag.split('.')
            targ_bag = "".join(targ_[:-2])+".bag"
            cmd = 'rosbag fix %s %s'%(active_bag, targ_bag)
            os.system(cmd)

        self.bagfile_idx    = 0
        self.bagfiles       = glob.glob('%s/*.bag' % self.bagfile_path)
        self.bagfiles.sort()
        self.bag            = rosbag
        self.bridge         = CvBridge()

        # create paths
        self.num_bagfiles = len(self.bagfiles)
        # now = datetime.datetime.now()
        # curr_time = now.strftime('%Y-%m-%d_%H-%M')
        self.out_depth_paths= []
        self.out_rgb_paths  = []
        self.out_pose_paths = []
       # self.out_map_paths  = []
        self.bagfile_cnt = 0

        self.readmetxt = '%s/readme.txt'% self.base_extraction_path
        f = open(self.readmetxt, 'w')
        f.write("metadata info file or metadata file contents the elements listed below \n")
        f.write("rgb_info.txt   contains:       idx, height, width, seq, time(s), time(ns) \n")
        f.write("depth_info.txt contains:       idx, height, width, seq, time(s), time(ns) \n")
        f.write("scan_info.txt  contains:       idx, rmin(m), rmax(m), ang_min, ang_max, ang_inc(rad), seq, time(s), time(ns) \n")
        f.write("odom contains (pose, twist):   idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz \n")
        f.write("odom_filt data contains:       idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz \n")

    def extractRGBD(self, bag, out_rgb_path, out_depth_path):
        cv_bridge = CvBridge()
        cnt = 0
        max_timediff_ms = 0
        sum_timediff_ms = 0
        depth_info_file = "%s/depth_info.txt" % out_depth_path
        rgb_info_file = "%s/rgb_info.txt" % out_rgb_path
        f_rgb = open(rgb_info_file, 'w')
        f_depth = open(depth_info_file, 'w')
        with tqdm(total=bag.get_message_count(self.rgbd_topic), position=0, leave=True) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.rgbd_topic]):
                pbar.update(1)
                # print("Size of the image: W {} x H {}".format(msg.width, msg.height))
                # print("Encoding of the frames: {}".format(msg.encoding))
                # sys.stdout.write('\r' + ('.' * cnt) + ' ')
                # sys.stdout.flush()
                # write info
                rgb_msg = msg.rgb
                depth_msg = msg.depth

                time_diff_ms = msg.timediff_ms.data
                sum_timediff_ms += time_diff_ms
                if(time_diff_ms > max_timediff_ms):
                    max_timediff_ms = time_diff_ms
                # process rgb
                f_rgb.write("%d %d %d " % (cnt, rgb_msg.height, rgb_msg.width))
                f_rgb.write("%d %d %d\n" % (rgb_msg.header.seq, rgb_msg.header.stamp.secs, rgb_msg.header.stamp.nsecs))
                cv_rgb = cv_bridge.imgmsg_to_cv2(img_msg=rgb_msg, desired_encoding="bgr8")
                rgb_file = "%s/%05d.png" % (out_rgb_path, cnt)
                cv2.imwrite(rgb_file, cv_rgb)

                # process depth
                f_depth.write("%d %d %d " % (cnt, depth_msg.height, depth_msg.width))
                f_depth.write("%d %d %d\n" % (depth_msg.header.seq, depth_msg.header.stamp.secs, depth_msg.header.stamp.nsecs))
                cv_depth = cv_bridge.imgmsg_to_cv2(img_msg=depth_msg, desired_encoding="passthrough")
                depth_file = "%s/%05d.png" % (out_depth_path, cnt)
                cv2.imwrite(depth_file, cv_depth)

                cnt += 1

        f_rgb.close()
        f_depth.close()
        if cnt > 0:
            print("\n avg rgb-d time diff: < %f > \n max rgb-d time diff: < %f >\n" % (sum_timediff_ms / cnt, max_timediff_ms) )

    def extractRGB(self, bag, out_rgb_path ):
        cv_bridge = CvBridge()
        cnt = 0
        info_file = "%s/rgb_info.txt" % out_rgb_path
        f = open(info_file, 'w')
        with tqdm(total = bag.get_message_count([self.rgb_topic]), position=0, leave=True) as pbar:
            for topic, msg, t in bag.read_messages(topics=[ self.rgb_topic ]):
                pbar.update(1)
                # print("Size of the image: W {} x H {}".format(msg.width, msg.height))
                # print("Encoding of the frames: {}".format(msg.encoding))
                # write info
                f.write("%d %d %d " % (cnt, msg.height, msg.width))
                f.write("%d %d %d\n" % (msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))

                cv_rgb = cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
                rgb_file = "%s/%05d.png" % (out_rgb_path, cnt)
                cv2.imwrite(rgb_file, cv_rgb)
                cnt += 1
        f.close()
        assert(cnt > 0 )

    def extractDepth(self, bag, out_depth_path ):
        cv_bridge = CvBridge()
        cnt = 0
        info_file = "%s/depth_info.txt" % out_depth_path
        f = open(info_file, 'w')
        with tqdm(total=bag.get_message_count(self.depth_topic), position=0, leave=True ) as pbar:
            for topic, msg, t in bag.read_messages(topics=[ self.depth_topic ]):
                pbar.update(1)
                # print("Size of the image: W {} x H {}".format(msg.width, msg.height))
                # print("Encoding of the frames: {}".format(msg.encoding))
                # sys.stdout.write('\r' + ('.' * cnt) + ' ')
                # sys.stdout.flush()
                # write info
                f.write("%d %d %d " % (cnt, msg.height, msg.width))
                f.write("%d %d %d\n" % (msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))

                cv_depth = cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="passthrough")
                depth_file = "%s/%05d.png" % (out_depth_path, cnt)
                cv2.imwrite(depth_file, cv_depth)
                cnt += 1
        f.close()
        assert(cnt > 0 )

    def extractScan(self, bag, out_scan_path):
        cnt = 0
        info_file = "%s/scan_info.txt" % out_scan_path
        f = open(info_file, 'w')
        with tqdm(total=bag.get_message_count(self.scan_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[ self.scan_topic ]):
                #print("intensities {}".format(msg.intensities))
                #print("Encoding of the frames: {}".format(msg.encoding))
                pbar.update(1)
                # write info
                f.write("%d %f %f %f %f %f %d %d %d \n" % (cnt, msg.angle_min, msg.angle_max, msg.angle_increment,
                                                        msg.range_min, msg.range_max, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs) )
                # f.write("%d %d %d\n" % (msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                # cv_rgb = cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

                scan_file = "%s/%05d.txt" % (out_scan_path, cnt)
                f_scan = open(scan_file, "w")
                f_scan.write('{} '.format( msg.ranges) )
                f_scan.write('\n')
                f_scan.close()
                cnt += 1
        f.close()

    def extractOdom(self, bag, out_traj_path):
        cnt = 0
        odom_file = "%s/odom.txt" % out_traj_path
        f = open(odom_file, 'w')
        with tqdm(total=bag.get_message_count(self.odom_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.odom_topic]):
                # print("intensities {}".format(msg.intensities))
                # print("Encoding of the frames: {}".format(msg.encoding))
                pbar.update(1)
                position = msg.pose.pose.position
                quat = msg.pose.pose.orientation
                linear = msg.twist.twist.linear
                angular = msg.twist.twist.angular

                # write info
                f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write("%f %f %f " % (position.x, position.y, position.z))
                f.write("%f %f %f %f " % (quat.x, quat.y, quat.z, quat.w))
                f.write("%f %f %f " % (linear.x, linear.y, linear.z))
                f.write("%f %f %f \n" % (angular.x, angular.y, angular.z))
                cnt += 1
            f.close()

    def extractOdomFilt(self, bag, out_traj_path):
        cnt = 0
        odom_file = "%s/odom_filt.txt" % out_traj_path
        f = open(odom_file, 'w')
        with tqdm(total=bag.get_message_count(self.odom_filt_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.odom_filt_topic]):
                # print("intensities {}".format(msg.intensities))
                # print("Encoding of the frames: {}".format(msg.encoding))
                pbar.update(1)
                position = msg.pose.pose.position
                quat = msg.pose.pose.orientation
                linear = msg.twist.twist.linear
                angular = msg.twist.twist.angular

                # write info
                f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write("%f %f %f " % (position.x, position.y, position.z))
                f.write("%f %f %f %f " % (quat.x, quat.y, quat.z, quat.w))
                f.write("%f %f %f " % (linear.x, linear.y, linear.z))
                f.write("%f %f %f \n" % (angular.x, angular.y, angular.z))
                cnt += 1
            f.close()

    def extractTwistStamped(self, bag, out_traj_path):
        cnt = 0
        twist_file = "%s/twist.txt" % out_traj_path
        f = open(twist_file, 'w')
        with tqdm(total=bag.get_message_count(self.twiststamped_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.twiststamped_topic]):
                # print("intensities {}".format(msg.intensities))
                # print("Encoding of the frames: {}".format(msg.encoding))
                pbar.update(1)
                linear = msg.twist.linear
                angular = msg.twist.angular

                # write info
                f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write("%f %f %f " % (linear.x, linear.y, linear.z))
                f.write("%f %f %f \n" % (angular.x, angular.y, angular.z))
                cnt += 1
            f.close()

    def runExtractor(self ):
        # for bagfile in bagfiles:
        #     # rosbag play each file then dump files into the dest folder

        self.extraction_path = '%s/%s' % (self.base_extraction_path, self.navtime_id)
        if os.path.isdir(self.extraction_path):
            shutil.rmtree(self.extraction_path)
        os.mkdir(self.extraction_path)

        num_bagfiles = len(self.bagfiles)
        with tqdm(total=num_bagfiles) as pbar:
            for bag_idx in tqdm( range( num_bagfiles), position=0, leave=True):
                bagfile = self.bagfiles[bag_idx]
                print("tot progress so far ...")
                pbar.update()
                # read first bag file to decode date time info
                stridx = bagfile.find('bag_')
                bagfile_time_str = bagfile[stridx + 4:-4]
                #bag_time = bagfile_time_str.split('-')
                bag_extraction_path = '%s/bag_%s' % (self.extraction_path, bagfile_time_str)
                if os.path.isdir(bag_extraction_path):
                    shutil.rmtree(bag_extraction_path)
                os.mkdir(bag_extraction_path)

                self.bag = rosbag.Bag(bagfile)

                out_traj_path   = '%s/traj' % (bag_extraction_path)
                out_depth_path  = '%s/depth'% (bag_extraction_path)
                out_rgb_path    = '%s/rgb'  % (bag_extraction_path)
                out_scan_path   = '%s/scan' % (bag_extraction_path)

                os.mkdir(out_traj_path)
                os.mkdir(out_depth_path)
                os.mkdir(out_rgb_path)
                os.mkdir(out_scan_path)

                print("extracting rgb-d msgs from <%d> th bag"%(bag_idx))
                self.extractRGBD(self.bag, out_rgb_path, out_depth_path)

                #print("extracting rgb of <%d> th bag: %s"%(bag_idx, bagfile_time_str))
                #self.extractRGB(self.bag, out_rgb_path)
                #print("extracting depth of <%d> th bag: %s"%(bag_idx, bagfile_time_str))
                #self.extractDepth(self.bag, out_depth_path)
                print("\r extracting scan msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractScan(self.bag, out_scan_path)
                print("\r extracting odom msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractOdom(self.bag, out_traj_path)
                print("\r extracting odom_filtered msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractOdomFilt(self.bag, out_traj_path)
                print("\r extracting cmd_vel(twist_stamped) from <%d> th bag: %s"%(bag_idx, bagfile_time_str) )
                self.extractTwistStamped(self.bag,out_traj_path)
        # slam pose, twist, etc
            #TODO
            # extractSLAMPose()
            # extractCmdVel()
