#! /usr/bin/env python
import sys
import os
import yaml
import datetime
import shutil
import numpy as np
import rospy
import rospkg
from pathlib import Path
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose2D


#from pynput.keyboard import Key, Listener

#def show(key):
 
    #print('\nYou Entered {0}'.format( key))
 
    #if key == Key.delete:
        #Stop listener
        #return False

def slam_publisher():

    rospack = rospkg.RosPack()
    base_dir = rospack.get_path('navdata_collector')
    
    rospy.init_node('slam_publisher', anonymous=True)
    #base_dir = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
    #base_dir = os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
    #    config_file = '/home/hankm/catkin_ws/src/navdata_collector/param/navdata_collector.yaml'
    config_file = '%s/param/navdata_collector.yaml' % base_dir


    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    listener = tf.TransformListener()
    world_frame_id = config['navdata_collector']['world_frame_id']
    base_frame_id = config['navdata_collector']['base_frame_id']
    rate = rospy.Rate(20)
    rpose_2d = Pose2D()
    pub = rospy.Publisher("slam_pose", Pose2D, queue_size=10)

    while not rospy.is_shutdown():
      
        try:
            (tran, rot) = listener.lookupTransform(world_frame_id, base_frame_id, rospy.Time(0))
            (roll, pit, yaw) = euler_from_quaternion(rot)
            rpose_2d.x = tran[0]
            rpose_2d.y = tran[1]
            rpose_2d.theta = yaw
            #print("robot pose: < %f %f %f >" % (rpose_2d.x, rpose_2d.y, rpose_2d.theta))
            pub.publish(rpose_2d)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    rate.sleep()

if __name__ == "__main__":
    try:
        slam_publisher()
    except rospy.ROSInterruptException:
        pass
