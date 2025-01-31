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
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from slam_toolbox_msgs.srv import SerializePoseGraph
from slam_toolbox_msgs.srv import SaveMap
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#import cv2
import roslaunch
import subprocess

_launch_sub1 = []
_launch_main = []
_pose_2d = Pose2D
_slam_pose = PoseWithCovarianceStamped
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

def mapsave_cb(msg):
    global _pose_2d

    if msg.data == True:
        print("Saving the current map and pose graph... \n")
        map_serializer = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
        resp1 = map_serializer(filename='out_map')
        save_map = rospy.ServiceProxy("/slam_toolbox/save_map", SaveMap)
        save_map(String('out_map'))
        pose_list = [_pose_2d.x, _pose_2d.y, _pose_2d.theta]
        rospy.set_param('/slam_toolbox/map_start_pose', pose_list)
        print("map start pos is set to %f %f %f"%(_pose_2d.x, _pose_2d.y, _pose_2d.theta ) )
    else:
        return

def pose_cb( msg ):
    global _slam_pose, _pose_2d
    #print("pos %f %f %f" %(msg.x, msg.y, msg.theta) )
    #_pos_2d = [msg.x, msg.y, msg.theta]
    _slam_pose = msg
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pit, yaw) = euler_from_quaternion(quat)
    _pose_2d.x = msg.pose.pose.position.x
    _pose_2d.y = msg.pose.pose.position.y
    _pose_2d.theta = yaw
    print("%f %f %f \n"% ( _pose_2d.x, _pose_2d.y, yaw) )
def main(argv):
    #global _pose_2d

    # map_serializer = rospy.ServiceProxy('/slam_toolbox/serialize_map', SerializePoseGraph)
    # resp1 = map_serializer(filename='out_map')
    global _slam_pub_process, _launch_main, _launch_sub
    rospy.init_node('mapper_launcher', anonymous=True)

    pos_sub = rospy.Subscriber("slam_toolbox/pose", PoseWithCovarianceStamped, pose_cb )
    mapsave_cmd_sub = rospy.Subscriber("save_the_current_map", Bool, mapsave_cb)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir

    # with open(config_file, "r") as f:
    #     config = yaml.safe_load(f)

    # ros_path = '%s/.ros' % Path.home()

    slam_pub_node = roslaunch.core.Node("navdata_collector", "slam_publisher_node.py")

    _launch_sub = roslaunch.scriptapi.ROSLaunch()
    _launch_sub.start()

    _slam_pub_process = _launch_sub.launch(slam_pub_node)
    print(_slam_pub_process.is_alive())

    roslaunch.configure_logging(uuid)
    _launch_main = roslaunch.parent.ROSLaunchParent(uuid, ['%s/launch/includes/start_slam.launch' % (base_dir)])
    _launch_main.start()

    rospy.on_shutdown( shutdown_hook )
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
