#! /usr/bin/env python
import sys
import os
import yaml
import datetime
import shutil
import numpy as np
import rospy
from pathlib import Path
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from move_base_msgs.msg import MoveBaseActionGoal
from navdata_collector.msg import scan_metadata
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from std_msgs.msg import Bool

#import cv2
import roslaunch

def pose_cb(msg):
    global launch1
    global launch2
    if msg.data is True:
        launch1.shutdown()
        launch2.shutdown()
        print("Terminating launch1 and launch2 \n")
    else:
        print("something wrong ... \n")

def main(argv):

    rospy.init_node('navdata_collector_launcher', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
#    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    # create working dir
    bagfile_root_path = config['navdata_collector']['bagfile_root_path']
    now = datetime.datetime.now()

    bagfile_path = '%s/%04d-%02d-%02d-%02d-%02d'%(bagfile_root_path,now.year, now.month, now.day, now.hour, now.minute)
    if os.path.isdir(bagfile_path):
        shutil.rmtree(bagfile_path)
    os.mkdir(bagfile_path)

    roslaunch.configure_logging(uuid)
    launch1 = roslaunch.parent.ROSLaunchParent(uuid, ['%s/launch/auto_collector_async.launch'%(base_dir)])

    cli_arg2 = ['%s/launch/includes/start_bag_async.launch'%base_dir, 'bagfile_path:=%s'%bagfile_path]
    roslaunch_args = cli_arg2[1:]
    roslaunch_file2 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg2)[0], roslaunch_args)]
    launch2 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file2)

    rate = rospy.Rate(10)

    #rospy.Subscriber("move_base/goal", MoveBaseActionGoal, goal_cb)
    rate.sleep()

    launch1.start()
    print("navdata_collector_async launcer is up \n")

    #rospy.Subscriber("/navdata_collector_is_initialized", Bool, navdata_start_cb, queue_size=10)
    data1 = rospy.wait_for_message('/navdata_collector_is_initialized', Bool, timeout= 15)
    print("in data %d"%data1.data)
    if data1.data:
        print("starting launch 2 \n")
        launch2.start()

    data2 = rospy.wait_for_message('/data_collection_is_completed', Bool, timeout=7200)
    if data2.data:
        print("Got data_collection_is_completed MSG \n")
        launch1.shutdown()
        launch2.shutdown()
        sys.exit()

    rospy.spin()

    #rospy.on_shutdown()
    print("navcollector task is completed \n")

if __name__ == "__main__":
    main(sys.argv)
