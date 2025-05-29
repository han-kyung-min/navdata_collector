#! /usr/bin/env python
import nav_msgs.msg
import sensor_msgs.msg
import rospy
import sys
import os
import yaml
import random
import time
import numpy as np
import tf2_ros
import tf
import datetime
import shutil
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import roslaunch


def main(argv):

    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    bagfile_root_path = config['navdata_collector']['bagfile_root_path']
    now = datetime.datetime.now()
    bagfile_path = '%s/%04d-%02d-%02d-%02d-%02d'%(bagfile_root_path,now.year, now.month, now.day, now.hour, now.minute)
    if os.path.isdir(bagfile_path):
        shutil.rmtree(bagfile_path)
    os.mkdir(bagfile_path)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.init_node('ae_data_collector_launcher', anonymous=True)

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    start = time.time()
    num_explorations = config['navdata_collector']['max_num_bagfiles']

    pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../'))
    catkin_dir = "%s/../"%pkg_dir

    print("waiting for core msgs ...  \n")
    try:
        rospy.wait_for_message(config['navdata_collector']['odom_topic'], nav_msgs.msg.Odometry, timeout=2)
        out_msg = "Got %s msg \n" % config['navdata_collector']['odom_topic']
        print('\033[32m' + out_msg + '\33[0m')
    except:
        out_msg = "It seems there is no  %s msg ... Please check your system \n" % config['navdata_collector']['odom_topic']
        print('\033[31m' + out_msg + '\33[0m')
        exit(-1)
    try:
        rospy.wait_for_message(config['navdata_collector']['rgb_topic'], sensor_msgs.msg.Image, timeout=5)
        out_msg = "Got %s msg \n" % config['navdata_collector']['rgb_topic']
        print('\033[32m' + out_msg + '\33[0m')
    except:
        out_msg = "It seems there is no %s msg ... Please check your system \n"%config['navdata_collector']['rgb_topic']
        print('\033[31m' + out_msg + '\33[0m')
        exit(-1)
    try:
        rospy.wait_for_message(config['navdata_collector']['depth_topic'], sensor_msgs.msg.Image, timeout=5)
        print("got depth msgs ! \n")
    except:
        out_msg = "It seems there is no %s msg ... Please check your system \n"%config['navdata_collector']['depth_topic']
        print('\033[33m' + out_msg + '\33[0m')
        exit(-1)

    out_msg = "I found all core msgs "
    print('\033[32m' + out_msg + '\33[0m')
    for round_idx in range(0, num_explorations):
        # make data dir

        # launch files
        roslaunch.configure_logging(uuid)
        launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/includes/move_former_slam.launch"%pkg_dir])
        launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/manual_collector_async.launch"%pkg_dir])

        cli_arg3 = ['%s/launch/includes/start_bag_async.launch' % base_dir, 'bagfile_path:=%s' % bagfile_path]
        roslaunch_args = cli_arg3[1:]
        roslaunch_file3 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg3)[0], roslaunch_args)]
        launch3 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file3)

        launch1.start()
        print(" <%d> th  move base & SLAM toolbox are up \n"%round_idx)
        time.sleep(5)
        t = rospy.Time(0)
        (trans, rot) = listener.lookupTransform("odom", "base_link", t)

        launch2.start()
        time.sleep(1)
        rospy.wait_for_message('map', nav_msgs.msg.OccupancyGrid, timeout=None)
        print("Got a map msg \n")
        launch3.start()
        print("<%d>th Bagging started \n"%round_idx )
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('exploration_is_done', Bool, timeout=None)
            print("exploration done? %d" % data.data)
            if data.data is True:
                break
        print("<%d> th exploration is done. Closing the exploration service \n"%round_idx)

        launch3.shutdown()
        launch2.shutdown()
        launch1.shutdown()
        #time.sleep(5)

    print("The data collection process is completed \n")
    end = time.time()
    print("Total data collection time ",  (end - start) )

if __name__ == '__main__':
    main(sys.argv)
