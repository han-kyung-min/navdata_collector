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
from sensor_msgs.msg import Joy
from navdata_collector.msg import rgbd

import roslaunch
import argparse

# Joystick trigger config
DEADMAN_BUTTON = 4   # Change if needed
AXIS_INDEX = 1
AXIS_THRESHOLD = 0.1
SHUTDOWN_DELAY = 5.0  # seconds

joy_msg = None
trigger_time = None
reached_goal_msg = None
finish_flag = None

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
    parser.add_argument("--map_file_name", type=str, required=True, help="Base path to saved map (no extension)")
    args = parser.parse_args(argv[1:])

    map_base = args.map_file_name
    
    print("map_file_name: %s"%map_base)
    
    global joy_msg, trigger_time, reached_goal_msg
    
    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../'))

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

    rospy.init_node('colldata_collection_launcher', anonymous=True)

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

    out_msg = "I've found all core msgs "
    print('\033[32m' + out_msg + '\33[0m')
    
    last_time = start #time.time() 

    #for round_idx in range(0, num_explorations):
        # make data dir
        # launch files
    roslaunch.configure_logging(uuid)
        
    cli_arg1 = [
        "%s/launch/includes/move_former_sync_slam.launch" % pkg_dir,
        "map_file_name:=%s" % map_base   # NOTE: := not =
    ]
    roslaunch1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg1)[0], cli_arg1[1:])]
    
    #launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/includes/move_former_slam.launch"%pkg_dir, "map_file=%s" %map_base] )
    launch1 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch1)
    
    launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/manual_collector_async.launch"%pkg_dir])

    cli_arg3 = ['%s/launch/includes/start_bag_async.launch' % base_dir, 'bagfile_path:=%s' % bagfile_path]
    roslaunch_args = cli_arg3[1:]
    roslaunch_file3 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg3)[0], roslaunch_args)]
    launch3 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file3)

    launch1.start()

    rospy.wait_for_message('map', nav_msgs.msg.OccupancyGrid, timeout=None)
    print("Got a map msg \n")
    
    t = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("odom", "base_link", t)

    launch2.start()
    rospy.wait_for_message('navdata_collector_is_initialized', Bool, timeout=None)

    launch3.start()
    rgbd_msg = rospy.wait_for_message('rgbd_throttle/rgbd', rgbd, timeout=None)
    print("got rgbd msgs ! \n")
    out_msg = "Got rgbd msg \n" 
    print('\033[32m' + out_msg + '\33[0m')
    
    rospy.Subscriber("/joy", Joy, joy_callback)
    print("navdata_collector_node is up.. listening to joy msgs.. \n")
    
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
            deadman = joy_msg.buttons[DEADMAN_BUTTON] == 1
            axis_active = ( abs(joy_msg.axes[0]) > AXIS_THRESHOLD or 
                          abs(joy_msg.axes[1]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[2]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[3]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[4]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[5]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[6]) > AXIS_THRESHOLD or
                          abs(joy_msg.axes[7]) > AXIS_THRESHOLD )
            if deadman and axis_active:
                #trigger_time = curr_time
                joy_flag_pub.publish(Bool(data=True))
                rospy.loginfo("Joystick condition detected. recording collision ..")
            else:
                joy_flag_pub.publish(Bool(data=False))
                
            if finish_flag is True:
                rospy.logwarn("Finishing cmd has been received. Shutting down...")
                break
            
            #if trigger_time is not None and curr_time - trigger_time > SHUTDOWN_DELAY:
                #rospy.logwarn("Joystick held for %.1f sec. Shutting down...", SHUTDOWN_DELAY)
                #break
            
                #triggered = True
        if(curr_time - start > max_nav_time):
            print("Max nav time has been reached %d \n"%max_nav_time)
            break
        if (reached_goal_msg is True):
            print("goal is reached")
            break

        rate.sleep()
        
    launch3.shutdown()
    launch2.shutdown()
    launch1.shutdown()
    time.sleep(1)

    rospy.loginfo("The data collection process is completed")
    end = time.time()
    print("Total data collection time\n",  (end - start) )

if __name__ == '__main__':
    main(sys.argv)
