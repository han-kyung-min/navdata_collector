#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 (Melodic/Noetic)
Launch slam_toolbox with a saved .posegraph, read Node0 pose from the optimized graph,
save to YAML, optionally publish /initialpose, then shut down slam_toolbox.

Usage:
  rosrun your_pkg load_map_and_save_node0.py \
      _posegraph:=/abs/path/to/map.posegraph \
      _out_yaml:=/abs/path/to/initial_pose.yaml \
      _publish_initialpose:=false
"""

from __future__ import print_function
import os, sys, math, yaml, time
import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# roslaunch (script API)
import roslaunch

NODES = []  # (id, x, y, yaw)

def marker_cb(msg):
    for m in msg.markers:
        # only ADD actions, skip line markers (edges)
        if m.action != 0:
            continue
        if m.type in (4, 5):  # LINE_LIST=4, LINE_STRIP=5
            continue
        q = (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w)
        yaw = euler_from_quaternion(q)[2]
        NODES.append((m.id, m.pose.position.x, m.pose.position.y, yaw))

def build_initialpose(x, y, yaw, frame_id="map",
                      cov_xy=0.25, cov_yaw=(10.0*math.pi/180.0)**2):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation = Quaternion(qx, qy, qz, qw)
    cov = [0.0]*36
    cov[0] = cov[7] = cov_xy
    cov[35] = cov_yaw
    msg.pose.covariance = cov
    return msg

def main():
    rospy.init_node("load_map_and_save_node0")

    posegraph = rospy.get_param("~posegraph", "")
    out_yaml  = rospy.get_param("~out_yaml", "initial_pose.yaml")
    publish_initialpose = bool(rospy.get_param("~publish_initialpose", False))
    initialpose_topic   = rospy.get_param("~initialpose_topic", "/initialpose")
    graph_topic         = rospy.get_param("~graph_topic", "/slam_toolbox/graph_visualization")
    slam_node_type      = rospy.get_param("~slam_node_type", "sync_slam_toolbox_node")
    map_frame           = rospy.get_param("~map_frame", "map")
    odom_frame          = rospy.get_param("~odom_frame", "odom")
    base_frame          = rospy.get_param("~base_frame", "base_link")
    startup_timeout_s   = float(rospy.get_param("~startup_timeout_s", 10.0))

    if not posegraph or not os.path.isfile(posegraph):
        rospy.logerr("~posegraph file not found: %s", posegraph)
        sys.exit(1)

    # Prepare roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [])  # no .launch files
    launch.start()

    # Set parameters under /slam_toolbox BEFORE launching the node
    # (so they appear as private params to that node)
    ns = "/slam_toolbox"
    rospy.set_param(ns + "/load_state_filename", posegraph)
    rospy.set_param(ns + "/localization", True)  # load only, don't change map
    rospy.set_param(ns + "/map_frame",  map_frame)
    rospy.set_param(ns + "/odom_frame", odom_frame)
    rospy.set_param(ns + "/base_frame", base_frame)
    rospy.set_param(ns + "/enable_interactive_mode", False)

    # Launch slam_toolbox
    node = roslaunch.core.Node(
        package="slam_toolbox",
        node_type=slam_node_type,  # "sync_slam_toolbox_node" (default)
        name="slam_toolbox",
        output="screen"
    )
    process = launch.launch(node)
    if not process.is_alive():
        rospy.logerr("Failed to start slam_toolbox.")
        launch.stop()
        sys.exit(1)

    # Subscribe to the graph markers
    sub = rospy.Subscriber(graph_topic, MarkerArray, marker_cb, queue_size=1)

    # Wait for markers to arrive
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < startup_timeout_s and not NODES:
        rate.sleep()

    if not NODES:
        rospy.logerr("No graph markers received on %s within %.1fs. "
                     "Check that the posegraph loaded correctly.", graph_topic, startup_timeout_s)
        process.stop(); launch.stop()
        sys.exit(1)

    # Pick smallest id as Node 0
    nid, x, y, yaw = sorted(NODES, key=lambda t: t[0])[0]

    # Save YAML
    data = {"frame_id": map_frame, "x": float(x), "y": float(y), "yaw": float(yaw)}
    with open(out_yaml, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False)
    rospy.loginfo("Saved Node%d as start pose -> %s  (x=%.3f, y=%.3f, yaw=%.3f rad)",
                  nid, out_yaml, x, y, yaw)

    # (Optional) publish /initialpose once
    if publish_initialpose:
        pub = rospy.Publisher(initialpose_topic, PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        # wait briefly for subscribers (e.g., slam_toolbox in your *next* run)
        for _ in range(50):
            if pub.get_num_connections() > 0:
                break
            rospy.sleep(0.1)
        pub.publish(build_initialpose(x, y, yaw, frame_id=map_frame))
        rospy.loginfo("Published %s", initialpose_topic)

    # Clean up slam_toolbox
    process.stop()
    launch.stop()
    rospy.signal_shutdown("done")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
