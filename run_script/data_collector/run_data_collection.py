#! /usr/bin/env python
import sys
import os
import pathlib
import yaml
import glob
import random
import time
import datetime
import shutil
import numpy as np

import rospy
import rosbag
import tf2_ros
import tf

from navdata_collector.msg import scan_metadata
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from std_msgs.msg import Bool

#import cv2
import roslaunch

def main(argv):

    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
#    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    # create working dir
    bagfile_root_path = config['navdata_collector']['bagfile_root_path']
    now = datetime.datetime.now()

    bagfile_path = '%s/%04d-%02d-%02d-%02d'%(bagfile_root_path,now.year, now.month, now.day, now.hour)
    if os.path.isdir(bagfile_path):
        shutil.rmtree(bagfile_path)
    os.mkdir(bagfile_path)

    # ros launch
    cmd = 'roslaunch navdata_collector navdata_collector.launch'
    os.system(cmd)


if __name__ == "__main__":
    main(sys.argv)