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

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from std_msgs.msg import Bool
from bag_extractor import bag_extractor
from metadata_syncer import metadata_syncer
#import cv2
import roslaunch
from roslaunch.parent import ROSLaunchParent

def main(argv):

    if(len(sys.argv) != 2):
        print( "usage: %s <config file>" % sys.argv[0] )
        return -1
    config_file = sys.argv[1]
    parent = ROSLaunchParent(run_id="decoding ros core", roslaunch_files=[], is_core=True)
    parent.start()

    rospy.init_node('bag_decoder', anonymous=True)

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    data_handler = bag_extractor(name='rosbag_data_extractor', **config)
    data_handler.runExtractor()
    parent.shutdown()

    print("data extraction is done. \nstart syncing the data")
    data_syncer = metadata_syncer(name= 'rosbag_data_syncer', **config)
    data_syncer.sync_metadata()

if __name__ == "__main__":
    main(sys.argv)

