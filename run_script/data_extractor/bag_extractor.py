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
from sensor_msgs.msg import LaserScan, Joy
from navdata_collector.msg import waypoint_stamped
import rigid_motion as rm
from tqdm import tqdm
from tqdm import trange

class bag_extractor():
    def __init__(self, name, **kwargs):
        self.name = name
        #self.rgb_topic      = kwargs['navdata_extractor']['rgb_topic']
        #self.depth_topic    = kwargs['navdata_extractor']['depth_topic']
        self.scan_topic     = kwargs['navdata_extractor']['scan_topic']
        self.odom_topic     = kwargs['navdata_extractor']['odom_topic']
        self.tf_topic       = kwargs['navdata_extractor']['tf_topic']
        self.odom_filt_topic= kwargs['navdata_extractor']['odom_filt_topic']
        self.slam_pose_topic= kwargs['navdata_collector']['robotpose_topic']

        self.rgbd_topic = None
        self.rgb_topic = None
        self.depth_topic = None
                
        self.bagfile_path   = kwargs['navdata_extractor']['inpath']  #bagfile_path   # source dir
        self.base_extraction_path   = kwargs['navdata_extractor']['outpath']
        self.navtime_id     = self.bagfile_path.split('/')[-1]

        self.config_extractor = kwargs['navdata_extractor']
        if os.path.exists(os.path.join(self.bagfile_path, "nav_data")):
            self.config_colldata_extractor = False
        elif os.path.exists(os.path.join(self.bagfile_path, "coll_data")):
            self.config_colldata_extractor = True
        else:
            raise RuntimeError("Expected either 'nav_data' or 'coll_data' folder in: {}".format(self.bagfile_path))

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

        if self.config_colldata_extractor:
            print("\033[38;5;208mCollision data extraction mode is on\033[0m")
            self.curr_rel_sg_topic = kwargs['colldata_extractor']['curr_rel_sg_topic']
            self.navdata_topic = kwargs['colldata_extractor']['navdata_topic']
            self.joy_topic = kwargs['colldata_extractor']['joy_topic']
        else:
            print("\033[38;5;51mOrdinary nav data extraction mode is on\033[0m")

        self.twiststamped_topic    = kwargs['navdata_extractor']['twiststamped_topic']



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
        f.write("tf_m2o.txt     contains:       idx, 0,   time(s), time(ns), px, py, pz, qx, qy, qz, qw \n")
        f.write("tf_m2b.txt     contains:       idx, 0,   time(s), time(ns), px, py, pz, qx, qy, qz, qw \n")

        if self.config_colldata_extractor:
            f.write('rel_subgoal.txt contains:  idx, seq, time(s), time(ns), px, py, pz, qx, qy, qz, qw \n')
            f.write('navdata.txt contains:      idx, seq, time(s), time(ns), is_joy_on, sg_idx, xy_dist, orient_dist, '
                                                'sg_px, sg_py, sg_qw, sg_qz, is_coll, '
                    '                           px1, py1, qw1, qz2, px2, py2, qw2, qz2, ... \n')
            f.write('joy.txt contains:          idx, seq, time(s), time(ns), axis0, axis1,...,axis7, button0, button1, ..., button12 \n'  )

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
                f.write("%d %d %d %d %f %f %f %f %f \n" % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,
                                                        msg.angle_min, msg.angle_max, msg.angle_increment,
                                                        msg.range_min, msg.range_max ) )
                # f.write("%d %d %d\n" % (msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                # cv_rgb = cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

                scan_file = "%s/%05d.txt" % (out_scan_path, cnt)
                f_scan = open(scan_file, "w")
                angle = msg.angle_min
                for r in msg.ranges:
                    f_scan.write('%.5f %.5f\n'% (angle, r))
                    angle += msg.angle_increment
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

    def extractTF(self, bag, out_traj_path):
        m2o_cnt = 0
        m2b_cnt = 0
        tf_m2o_file = '%s/tf_m2o.txt' % out_traj_path
        tf_m2b_file = '%s/tf_m2b.txt' % out_traj_path
        #target_frames = ["odom", "base_link"]
        f_m2o = open(tf_m2o_file, 'w')
        f_m2b = open(tf_m2b_file, 'w')
        mHo = np.eye(4)
        mHb = np.eye(4)
        with tqdm(total=bag.get_message_count(self.tf_topic)) as pbar:
            for topic, msg, i in bag.read_messages(topics=[self.tf_topic]):
                pbar.update(1)
                for tform in msg.transforms:
                    parent = tform.header.frame_id
                    child  = tform.child_frame_id
                    stamp  = tform.header.stamp

                    if parent == "map" and child == "odom":
                        x_o = tform.transform.translation.x
                        y_o = tform.transform.translation.y
                        qx_o = tform.transform.rotation.x
                        qy_o = tform.transform.rotation.y
                        qz_o = tform.transform.rotation.z
                        qw_o = tform.transform.rotation.w

                        mHo = rm.quat_to_htm( [qw_o, qx_o, qy_o, qz_o] )
                        mHo[:2, 3] = [x_o, y_o]
                        f_m2o.write("%d %d %d %d %f %f %f %f %f %f %f \n" % (m2o_cnt, 0, stamp.secs, stamp.nsecs, x_o, y_o, 0, qx_o, qy_o, qz_o, qw_o))
                        m2o_cnt += 1

                    elif parent == "odom" and child == "base_link":
                        x_b = tform.transform.translation.x
                        y_b = tform.transform.translation.y
                        qx_b = tform.transform.rotation.x
                        qy_b = tform.transform.rotation.y
                        qz_b = tform.transform.rotation.z
                        qw_b = tform.transform.rotation.w
                        oHb = rm.quat_to_htm( [qw_b, qx_b, qy_b, qz_b] )
                        oHb[:2, 3] = [x_b, y_b]
                        mHb = np.matmul(mHo, oHb)
                        q = rm.htm_to_quat( mHb )   # w, x, y, z
                        x, y, z = mHb[:3, 3]
                        f_m2b.write("%d %d %d %d %f %f %f %f %f %f %f \n" % (m2b_cnt, 0, stamp.secs, stamp.nsecs, x, y, z, q[1], q[2], q[3], q[0]))
                        m2b_cnt += 1

################################# colldata extracotr ###################################

    def extractSubGoals(self, bag, out_traj_path):
        cnt = 0
        sg_file = "%s/rel_subgoal.txt" % out_traj_path
        f = open(sg_file, 'w')
        with tqdm(total=bag.get_message_count(self.curr_rel_sg_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.curr_rel_sg_topic]):
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
                f.write("%f %f %f %f \n" % (quat.x, quat.y, quat.z, quat.w))
                # f.write("%f %f %f " % (linear.x, linear.y, linear.z))
                # f.write("%f %f %f \n" % (angular.x, angular.y, angular.z))
                cnt += 1
            f.close()

    def extractNavData(self, bag, out_traj_path):
        cnt = 0
        navdata_file = "%s/navdata.txt" % out_traj_path
        f = open(navdata_file, 'w')
        with tqdm(total=bag.get_message_count(self.navdata_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.navdata_topic]):
                # print("intensities {}".format(msg.intensities))
                # print("Encoding of the frames: {}".format(msg.encoding))
                pbar.update(1)
                is_joy_on = msg.joystick.data

                curr_sg_idx = msg.sg_idx.data
                is_collision = msg.is_collision.data
                pose_diff = msg.pose_diff.data

                data_raw = np.array(msg.waypoints.data, dtype=np.float32)
                dims = msg.waypoints.layout.dim
                rows = dims[0].size
                cols = dims[1].size
                np_waypoint = data_raw.reshape((rows, cols))
                xy_dist = msg.xy_dist.data
                orient_dist = msg.orient_dist.data
                # write info
                f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write("%d " % (is_joy_on))
                f.write("%d " % (curr_sg_idx))
                f.write("%f %f "% (xy_dist, orient_dist))
                f.write("%f %f %f %f "%(pose_diff[0], pose_diff[1], pose_diff[2], pose_diff[3]))
                f.write("%d "%is_collision)
                for ii in range(0, rows):
                    x, y, qw, qz = np_waypoint[ii]
                    f.write("%f %f %f %f " % (x, y, qw, qz))
                f.write("\n")
                cnt += 1
            f.close()

    # def extractWayPoints(self, bag, out_traj_path):
    #     cnt = 0
    #     waypoint_file = "%s/waypoints.txt" % out_traj_path
    #     f = open(waypoint_file, 'w')
    #     with tqdm(total=bag.get_message_count(self.waypoint_topic)) as pbar:
    #         for topic, msg, t in bag.read_messages(topics=[self.waypoint_topic]):
    #             # print("intensities {}".format(msg.intensities))
    #             # print("Encoding of the frames: {}".format(msg.encoding))
    #             pbar.update(1)
    #             is_joy_on = msg.joystick.data
    #             curr_sg_idx = msg.sg_idx.data
    #             data_raw = np.array(msg.waypoints.data, dtype=np.float32)
    #             dims = msg.waypoints.layout.dim
    #             rows = dims[0].size
    #             cols = dims[1].size
    #             np_waypoint = data_raw.reshape((rows, cols))
    #             xy_dist = msg.xy_dist.data
    #             orient_dist = msg.orient_dist.data
    #             # write info
    #             f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
    #             f.write("%d " % (is_joy_on))
    #             f.write("%d " % (curr_sg_idx))
    #             f.write('%f %f '% (xy_dist, orient_dist))
    #             for ii in range(0, rows):
    #                 x, y, qw, qz = np_waypoint[ii]
    #                 f.write("%f %f %f %f " % (x, y, qw, qz))
    #             f.write("\n")
    #             cnt += 1
    #         f.close()

    def extractJoyMsgs(self, bag, out_traj_path):
        cnt = 0
        sg_file = "%s/joy.txt" % out_traj_path
        f = open(sg_file, 'w')
        with tqdm(total=bag.get_message_count(self.joy_topic)) as pbar:
            for topic, msg, t in bag.read_messages(topics=[self.joy_topic]):
                pbar.update(1)

                axes_str = ' '.join(f'{a:.3f}' for a in msg.axes)
                buttons_str = ' '.join(str(b) for b in msg.buttons)
                # write info
                f.write("%d %d %d %d " % (cnt, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write(f"{axes_str} {buttons_str}\n")
                cnt += 1
            f.close()


    def runExtractor(self ):
        # for bagfile in bagfiles:
        #     # rosbag play each file then dump files into the dest folder

        if self.config_colldata_extractor:
            self.extraction_path = '%s/coll_%s' % (self.base_extraction_path, self.navtime_id)
        else:
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

                # copy slam map if exist
                data_file = glob.glob('%s/*.data'%self.bagfile_path)
                pgo_file = glob.glob('%s/*.posegraph'%self.bagfile_path)
                map_img_file = glob.glob('%s/*.png'%self.bagfile_path)
                map_yaml_file = glob.glob('%s/*.yaml'%self.bagfile_path)

                target_bag_file_path = glob.glob('%s/bag_*'%self.bagfile_path)
                print(self.bagfile_path)
                print(target_bag_file_path)
                target_bag_file = target_bag_file_path[0].rstrip('/').split('/')[-1]
                #print(target_bag_file)
                if data_file:
                    shutil.copy(data_file[0], '%s/map.data'%bag_extraction_path)
                if pgo_file:
                    shutil.copy(pgo_file[0], '%s/map.posegraph'%bag_extraction_path)
                if map_img_file:
                    shutil.copy(map_img_file[0], '%s/slam_map.png'%bag_extraction_path)
                if map_yaml_file:
                    shutil.copy(map_yaml_file[0], '%s/slam_map.yaml' % bag_extraction_path)

                #if target_bag_file:
                #    shutil.copy(target_bag_file_path[0], '%s/%s'%(bag_extraction_path, target_bag_file) )

                out_traj_path   = '%s/traj' % (bag_extraction_path)
                out_depth_path  = '%s/depth'% (bag_extraction_path)
                out_rgb_path    = '%s/rgb'  % (bag_extraction_path)
                out_scan_path   = '%s/scan' % (bag_extraction_path)
                os.mkdir(out_traj_path)
                os.mkdir(out_depth_path)
                os.mkdir(out_rgb_path)
                os.mkdir(out_scan_path)

                bag_id = bagfile.split('/')[-1].split('.')[0]
                pathlib.Path('%s/%s' % (bag_extraction_path, bag_id)).touch()

                print("extracting rgb-d msgs from <%d> th bag"%(bag_idx))
                self.extractRGBD(self.bag, out_rgb_path, out_depth_path)

                print("\r extracting TF (base_link wrt map) from <%d> th bag: %s"%(bag_idx, bagfile_time_str) )
                self.extractTF(self.bag, out_traj_path)

                print("\r extracting scan msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractScan(self.bag, out_scan_path)
                print("\r extracting odom msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractOdom(self.bag, out_traj_path)
                print("\r extracting odom_filtered msgs from <%d> th bag: %s"% (bag_idx, bagfile_time_str) )
                self.extractOdomFilt(self.bag, out_traj_path)
                print("\r extracting cmd_vel(twist_stamped) from <%d> th bag: %s"%(bag_idx, bagfile_time_str) )
                self.extractTwistStamped(self.bag, out_traj_path)

                #print("\r extracting map from <%d> th bag: %s"%(bag_idx, bagfile_time_str) )
                #self.extractMap(self.bag, out_traj_path)

                if self.config_colldata_extractor:
                    # For collision data collection
                    out_sg_path     = '%s/rel_subgoals' % (bag_extraction_path)
                    out_navdata_path     = '%s/navdata' % (bag_extraction_path)
                    out_joy_path    = '%s/joy' % (bag_extraction_path)
                    os.mkdir(out_sg_path)
                    os.mkdir(out_navdata_path)
                    os.mkdir(out_joy_path)
                    print("\r extracting curr relative sg from <%d> th bag: %s" % (bag_idx, bagfile_time_str))
                    self.extractSubGoals(self.bag, out_sg_path)
                    print("\r extracting waypoints from <%d> th bag: %s"%(bag_idx, bagfile_time_str))
                    self.extractNavData(self.bag, out_navdata_path)
                    print("\r extracting joy cmds from <%d> th bag: %s" % (bag_idx, bagfile_time_str))
                    self.extractJoyMsgs(self.bag, out_joy_path)

        # slam pose, twist, etc
            #TODO
            # extractSLAMPose()
            # extractCmdVel()
