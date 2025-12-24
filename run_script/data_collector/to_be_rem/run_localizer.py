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
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from slam_toolbox_msgs.srv import SerializePoseGraph
from slam_toolbox_msgs.srv import SaveMap
#import cv2
import roslaunch
import subprocess

_launch_sub1 = []
_launch_main = []
_pos_2d = []
_slam_pub_process = []
_launch1 = roslaunch
def shutdown_hook(  ):
    global _slam_pub_process
    global _launch_sub, _launch_main
    print("shutdown time! Saving maps")
    _slam_pub_process.stop()
    _launch_main.shutdown()
    # map_serializer = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
    # resp1 = map_serializer(filename='out_map')


def pose_cb( msg ):
    global _pos_2d
    #print("pos %f %f %f" %(msg.x, msg.y, msg.theta) )
    _pos_2d = [msg.x, msg.y, msg.theta]

def main(argv):
    #global _pose_2d

    # map_serializer = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
    # resp1 = map_serializer(filename='out_map')
    global _slam_pub_process, _launch_main, _launch_sub
    rospy.init_node('mapper_launcher', anonymous=True)

    pos_sub = rospy.Subscriber("slam_pose", Pose2D, pose_cb )

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir

    # with open(config_file, "r") as f:
    #     config = yaml.safe_load(f)
    # ros_path = '%s/.ros' % Path.home()

#    init_pose_pub = rospy.Publisher('initialpose', PoseWithCo )

    slam_pub_node = roslaunch.core.Node("navdata_collector", "slam_publisher_node.py")

    _launch_sub = roslaunch.scriptapi.ROSLaunch()
    _launch_sub.start()

    _slam_pub_process = _launch_sub.launch(slam_pub_node)
    print(_slam_pub_process.is_alive())

    roslaunch.configure_logging(uuid)
    _launch_main = roslaunch.parent.ROSLaunchParent(uuid, ['%s/launch/includes/start_localization.launch' % (base_dir)])
    _launch_main.start()

    rospy.on_shutdown( shutdown_hook )
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
