#! /usr/bin/env python
import nav_msgs.msg
import sensor_msgs.msg
import rospy
import rospkg
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
from pathlib import Path
from PIL import Image as PILImage
#from datetime import datetime

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from navdata_collector.msg import rgbd
from nav_msgs.msg import OccupancyGrid
from slam_toolbox_msgs.srv import SerializePoseGraph, SaveMap
import roslaunch

import signal, subprocess, select

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def save_initial_pose(output_file="initial_pose.yaml",
                      publish_now=True,
                      frame_map="map",
                      frame_base="base_link"):
    tf_listener = TransformListener()

    rospy.loginfo("Waiting for TF %s -> %s..." % (frame_map, frame_base))
    tf_listener.waitForTransform(frame_map, frame_base, rospy.Time(0), rospy.Duration(10.0))

    (trans, rot) = tf_listener.lookupTransform(frame_map, frame_base, rospy.Time(0))
    roll, pitch, yaw = euler_from_quaternion(rot)

    pose_data = {
        "frame_id": frame_map,
        "x": float(trans[0]),
        "y": float(trans[1]),
        "yaw": float(yaw)
    }
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, "w") as f:
        yaml.dump(pose_data, f, default_flow_style=False)

    rospy.loginfo("Saved initial pose to %s" % os.path.abspath(output_file))
    rospy.loginfo("Pose: x=%.3f, y=%.3f, yaw=%.3f rad" %
                  (pose_data["x"], pose_data["y"], pose_data["yaw"]))

    if publish_now:
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        rospy.sleep(1.0)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, pose_data["yaw"])
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_map
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = pose_data["x"]
        msg.pose.pose.position.y = pose_data["y"]
        msg.pose.pose.orientation = Quaternion(qx, qy, qz, qw)

        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = (10.0 * math.pi/180.0) ** 2
        msg.pose.covariance = cov
        pub.publish(msg)
        rospy.loginfo("Published /initialpose")

    return pose_data

def save_slamtoolbox_maps_cli(base_path, slam_ns="/slam_toolbox"):
    serialize_srv = slam_ns + "/serialize_map"
    savemap_srv   = slam_ns + "/save_map"
    os.makedirs(os.path.dirname(base_path), exist_ok=True)
    def call(cmd): subprocess.check_call(cmd, shell=True)
    call(f'rosservice call {serialize_srv} "filename: \'{base_path}\'"')
    call(f'rosservice call {savemap_srv}   "name:     \'{base_path}\'"')

def key_pressed():
    dr,_,_ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None

def shutdown_and_wait(launch_obj, name):
    print(f"Shutting down {name}...")
    try:
        launch_obj.shutdown()
        while any(p.is_alive() for p in launch_obj._processes.values()):
            time.sleep(0.1)  # short poll, no long sleeps
    except Exception as e:
        print(f"[WARN] Failed to shutdown {name}: {e}")


def save_map(base_path, msg):
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    img = np.zeros((h, w), dtype=np.uint8)
    img[data == 0] = 254
    img[data == 100] = 0
    img[data == -1] = 205

    #map_file = datetime.now().strftime(f"%Y-%m-%d-%H-%M.png")
    map_file_path = base_path + "/slam_map.png" #+ map_file

    img = np.flipud(img)
    PILImage.fromarray(img, mode="L").save(map_file_path)

    meta = {
        "image": "slam_map.png",
        "resolution": msg.info.resolution,
        "origin": [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            0.0
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }
    yaml_file_path = base_path + "/slam_map.yaml"

    with open(yaml_file_path, "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def main(argv):

    home_dir = os.path.expanduser("~")
    catkin_dir = os.path.join(home_dir, "catkin_ws")
    pkg = 'navdata_collector'
    share_path = rospkg.RosPack().get_path(pkg)              # …/install/share/navdata_collector
    #install = os.path.dirname(os.path.dirname(share_path))  # …/install (or …/devel)
    #base_dir = os.path.dirname(install)              # …/catkin_ws
    pkg_lib_dir = os.path.join(catkin_dir, 'install/lib', pkg)
    pkg_share_dir = os.path.join(catkin_dir, 'install/share', pkg)

    config_file = '%s/param/navdata_collector.yaml' % pkg_share_dir
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

    rospy.init_node('manual_data_collector_launcher', anonymous=True)

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    start = time.time()
    
    num_explorations = config['navdata_collector']['max_num_bagfiles']
    max_time_per_round = config['navdata_collector']['max_time_per_round']
    max_nav_time = config['navdata_collector']['max_nav_time']

    print("waiting for core msgs ...  \n")
    odom_msg = None
    try:
        odom_msg = rospy.wait_for_message(config['navdata_collector']['odom_topic'], nav_msgs.msg.Odometry, timeout=2)
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

    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    print("odom: %.3f (m) %.3f (m)" % (x, y))

    if abs(x) > 0. and abs(y) > 0.:
        answer = input("----------------------------------------------------------------\n\n\n"
                       "\tDid you reboot the robot ? (y/n): \n\n\n"
                       "----------------------------------------------------------------\n")

        if answer.lower() == "y":
            print("Stepping to next")
        else:
            print("Make sure to reboot the robot to reset odom (0,0,0) \n")
            exit(0)

    out_msg = "I found all core msgs "
    print('\033[32m' + out_msg + '\33[0m')
    
    #last_time = start #time.time()

    for round_idx in range(0, num_explorations):

            # make data dir
        # launch files
        last_time = time.time()
        roslaunch.configure_logging(uuid)
        launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/includes/move_former_slam.launch"%pkg_share_dir])
        launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["%s/launch/manual_collector_async.launch"%pkg_share_dir])

        cli_arg3 = ['%s/launch/includes/start_bag_async.launch' % pkg_share_dir, 'bagfile_path:=%s' % bagfile_path]

        roslaunch_args = cli_arg3[1:]
        roslaunch_file3 = [(roslaunch.rlutil.resolve_launch_arguments(cli_arg3)[0], roslaunch_args)]
        launch3 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file3)

        launch1.start()
        print(" <%d> th  move base & SLAM toolbox are up \n"%round_idx)
        rospy.wait_for_message('map', nav_msgs.msg.OccupancyGrid, timeout=None)
        print("Got a map msg \n")
        
        t = rospy.Time(0)
        (trans, rot) = listener.lookupTransform("odom", "base_link", t)

        launch2.start()
        rospy.wait_for_message('navdata_collector_is_initialized', Bool, timeout=None)
        print("navdata_collector_node is up \n")
        
        launch3.start()
        rgbd_msg = rospy.wait_for_message('rgbd_throttle/rgbd', rgbd, timeout=None)
        
        print("<%d>th Bagging started \n"%round_idx )
        print("\033[36mPress 'q' followed by 'enter' key when you want to finish the data collection \n\033[0m")

        #init_pose_file = '%s/init_pose.yaml'%bagfile_path
        # pose = save_initial_pose(
        #     output_file=init_pose_file,
        #     publish_now=True,
        #     frame_map="map",
        #     frame_base="base_link"
        # )

        while True:
            ch = key_pressed()
            curr_time = time.time()
            if ch == 'q':
                print("Ending round on 'q'.")
                break
            if curr_time - last_time > max_time_per_round:
                print(f"Time limit {max_time_per_round}s reached.")
                break
            time.sleep(0.05)

        print("<%d> th exploration is done. Closing the exploration service \n"%round_idx)
        map_base = os.path.join(bagfile_path, "MAP_round_%02d" % round_idx)
        data_type_file = os.path.join(bagfile_path, "nav_data")
        data_type_file = Path(data_type_file)  # convert str to Path
        data_type_file.touch(exist_ok=True)
        # Save SLAM maps BEFORE stopping slam_toolbox
        try:
            print("[INFO] Saving map to:", map_base)
            save_slamtoolbox_maps_cli(map_base, slam_ns="/slam_toolbox")  # adjust namespace if you use one
            print("[INFO] Map saved.")
        except Exception as e:
            print("[WARN] Map save failed:", e)

        # Save SLAM map img
        msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=5.0)
        save_map(bagfile_path, msg)

        shutdown_and_wait(launch3, "rosbag recorder")
        shutdown_and_wait(launch2, "data collector")
        shutdown_and_wait(launch1, "SLAM + move_base")

        # launch3.shutdown()
        # launch2.shutdown()
        # launch1.shutdown()
        #time.sleep(5)

        if(curr_time - start > max_nav_time):
            print("Max nav time has been reached %d \n"%max_nav_time)
            break

    print("The data collection process is completed \n")
    end = time.time()
    print("Total data collection time ",  (end - start) )

if __name__ == '__main__':
    main(sys.argv)
