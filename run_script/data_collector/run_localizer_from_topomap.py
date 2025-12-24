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
import math

from pathlib import Path
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from navdata_collector.msg import rgbd

import roslaunch
import argparse

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

# Joystick trigger config
DEADMAN_BUTTON = 4   # Change if needed
AXIS_INDEX = 1
AXIS_THRESHOLD = 0.1
SHUTDOWN_DELAY = 5.0  # seconds

joy_msg = None
trigger_time = None
reached_goal_msg = None
finish_flag = None

def reset_slam_pose(topomap_dir, ns="/slam_toolbox", mapping_mode=True):
    """
    Set params so SLAM Toolbox starts at your saved init pose in MAPPING mode.
    Call this BEFORE launching slam_toolbox.
    """
    slam_pose_file = '%s/slam_poses.txt'%topomap_dir
    poses = np.loadtxt(slam_pose_file)
    [x, y, yaw] = poses[0][1:]
    print("Pose @ node 0:  (%f  %f  %f) \n"%(x, y, yaw) )

    pfx = '%s/map'%topomap_dir  #_prefix(posegraph)
    if not (os.path.isfile(pfx+".posegraph") and os.path.isfile(pfx+".data")):
        raise IOError("Missing map files: %s.posegraph / %s.data" % (pfx, pfx))

    def P(k): return ns.rstrip("/") + "/" + k

    # Load serialized map and seed pose
    rospy.set_param(P("map_file_name"), pfx)             # prefix (no extension)
    rospy.set_param(P("map_start_pose"), [float(x), float(y), float(yaw)])    # x, y, yaw(rad)
    rospy.set_param(P("map_start_at_dock"), False)       # don't override with Node0

    # Choose mode: mapping vs localization
    rospy.set_param(P("localization"), not mapping_mode) # False => mapping
    rospy.set_param(P("enable_interactive_mode"), False)
    rospy.sleep(0.1)
    
    return {"x": x, "y": y, "yaw": yaw, "prefix": pfx, "ns": ns}

def joy_callback(msg):
    global joy_msg
    joy_msg = msg

def goal_callback(msg):
    global reached_goal_msg
    reached_goal_msg = msg

def finish_callback(msg):
    global finish_flag
    finish_flag = msg.data

def main(argv):
    
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topomap_dir",
        type=str,
        required=True,
        help="Base path to saved map (no extension)"
    )

    args = parser.parse_args(argv[1:])
    topomap_dir = args.topomap_dir

    map_base = '%s/map'%topomap_dir
    init_pose_file = '%s/init_pose'%topomap_dir
    print("map_file_name: %s"%map_base)
    print("init_pose file: %s"%init_pose_file )
    
    global joy_msg, trigger_time, reached_goal_msg
    
    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    
    #base_dir = os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
    print(base_dir)
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../'))

    config_file = '%s/param/navdata_collector.yaml' % base_dir    
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.init_node('slam_toolbox_localizer_launcher', anonymous=True)

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    start = time.time()
    
    #num_explorations = config['navdata_collector']['max_num_bagfiles']
    max_time_per_round = config['navdata_collector']['max_time_per_round']
    max_nav_time = config['navdata_collector']['max_nav_time']

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
   
    last_time = start #time.time() 

    roslaunch.configure_logging(uuid)
        
    cli_arg1 = [
        "%s/launch/includes/move_former_sync_slam.launch" % pkg_dir,
        "map_file_name:=%s" % map_base   # NOTE: := not =
    ]
    roslaunch1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg1)[0], cli_arg1[1:])]
    
    #launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/includes/move_former_slam.launch"%pkg_dir, "map_file=%s" %map_base] )
    launch1 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch1)
    
    reset_slam_pose(topomap_dir=topomap_dir, ns="/slam_toolbox", mapping_mode=True)

    launch1.start()

    rospy.wait_for_message('map', nav_msgs.msg.OccupancyGrid, timeout=None)
    print("Got a map msg \n")
    
    t = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("odom", "base_link", t)

    if rospy.has_param("/slam_toolbox"):
        rospy.delete_param("/slam_toolbox")
        rospy.sleep(0.1)  # tiny guard
 
    rospy.Subscriber("/joy", Joy, joy_callback)
    print("localizatoin node is up.. \n")
    
    joy_flag_pub = rospy.Publisher("/joy_flag", Bool, queue_size=1)
    process_start_pub = rospy.Publisher("/navdata_collector/colldata_collection_node_is_up", Bool,queue_size=1, latch=True) 
    process_start_pub.publish(Bool(data=True))
    
    goal_sub = rospy.Subscriber("/topoplan/reached_goal", Bool, goal_callback )
    finish_sub = rospy.Subscriber("/finish_data_collection", Bool, finish_callback)
    
    trigger_time = None
    while not rospy.is_shutdown():
        curr_time = time.time()
        if( curr_time - last_time > max_time_per_round  ):
            last_time = curr_time
            print("This data collection process reached the max time: <%d> "% max_time_per_round)
            break
        
        if joy_msg:
               
            if finish_flag is True:
                rospy.logwarn("Finishing cmd has been received. Shutting down...")
                break
            
        if (reached_goal_msg is True):
            print("goal is reached")
            break

        rate.sleep()
        
    launch1.shutdown()
    time.sleep(1)

    rospy.loginfo("The localizatoin node is terminated")
    end = time.time()

if __name__ == '__main__':
    main(sys.argv)
