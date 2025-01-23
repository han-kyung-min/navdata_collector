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
from comp_data_handler import comp_data_handler

#import cv2
import roslaunch

def main(argv):

    if(len(sys.argv) != 2):
        print( "usage: %s <bagfile_path>" % sys.argv[0] )
        return -1
    bagfile_path = sys.argv[1]
    print("bagfile_path: %s"%bagfile_path)
    rospy.init_node('bag_decoder', anonymous=True)
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    proj_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
    #args = parser.parse_args()

    # config_file = '%s/param/navdata_collector.yaml'%proj_dir
    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    data_handler = comp_data_handler(name='comp_data_handler', bagfile_path=bagfile_path, **config)

    data_handler.runExtractor()

if __name__ == "__main__":
    main(sys.argv)

