import os, sys, math, yaml, argparse, time
import rospy, roslaunch
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from tf.transformations import euler_from_quaternion

def main():
    ap = argparse.ArgumentParser(description="Load SLAM map and save Node0 init pose (x,y,yaw).")
    ap.add_argument("--map-prefix", required=True, help="Path to map prefix (no extension). Ex: /path/to/map")
    ap.add_argument("--out-yaml",   required=True, help="Where to write init_pose.yaml")
    ap.add_argument("--ns", default="/slam_toolbox", help="slam_toolbox namespace (default: /slam_toolbox)")
    ap.add_argument("--timeout", type=float, default=30.0, help="Seconds to wait for pose")
    args = ap.parse_args()

    pre = args.map_prefix
    if not (os.path.isfile(pre+".posegraph") and os.path.isfile(pre+".data")):
        sys.stderr.write("Missing files: %s.posegraph or %s.data\n" % (pre, pre)); sys.exit(1)

    rospy.init_node("save_init_pose_min", anonymous=True)

    # Set params BEFORE starting node
    def P(k): return args.ns.rstrip("/")+"/"+k
    rospy.set_param(P("localization"), True)
    rospy.set_param(P("map_file_name"), pre)       # prefix only
    rospy.set_param(P("map_start_at_dock"), True)  # start at Node 0
    # (frames & scan can come from your launch; not required here)

    # Launch localization node
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launcher = roslaunch.scriptapi.ROSLaunch()
    launcher.parent = roslaunch.parent.ROSLaunchParent(uuid, [])
    launcher.start()

    ns_path, node_name = os.path.split(args.ns.rstrip("/"))
    if ns_path == "": ns_path = "/"
    node = roslaunch.core.Node("slam_toolbox", "localization_slam_toolbox_node",
                               name=node_name,
                               namespace=(None if ns_path=="/" else ns_path),
                               output="screen")
    proc = launcher.launch(node)
    if not proc.is_alive():
        launcher.stop(); sys.stderr.write("Failed to start slam_toolbox\n"); sys.exit(1)

    # Compose topic path and read ONE pose message (supports both types)
    topic = ("" if ns_path=="/" else ns_path)+"/"+node_name+"/pose"
    x=y=yaw=None
    deadline = time.time()+args.timeout
    try:
        # Try PoseWithCovarianceStamped first
        msg = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=max(1.0, args.timeout/2.0))
        q = msg.pose.pose.orientation
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    except rospy.ROSException:
        print("doing the exception  !! \n")
        # Fallback: Pose2D
        remaining = max(1.0, deadline - time.time())
        msg = rospy.wait_for_message(topic, Pose2D, timeout=remaining)
        x, y, yaw = float(msg.x), float(msg.y), float(msg.theta)

    # Save YAML
    out = os.path.abspath(args.out_yaml)
    with open(out, "w") as f:
        yaml.safe_dump({"frame_id":"map","x":x,"y":y,"yaw":yaw}, f, default_flow_style=False)
    print("Saved init pose -> %s  (x=%.3f, y=%.3f, yaw=%.3f rad)" % (out, x, y, yaw))

    # Clean up
    proc.stop(); launcher.stop()

if __name__ == "__main__":
    main()
    
    
    
    
    
#from __future__ import print_function
#import os, sys, time, math, yaml, rospy, roslaunch
#from visualization_msgs.msg import MarkerArray
#from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from slam_toolbox_msgs.srv import DeserializePoseGraph, DeserializePoseGraphRequest



#NODES = []

#def marker_cb(msg):
    #for m in msg.markers:
        #if m.action != 0:       # only ADD
            #continue
        #if m.type in (4, 5):    # LINE_LIST/LINE_STRIP = edges
            #continue
        #q = (m.pose.orientation.x, m.pose.orientation.y,
             #m.pose.orientation.z, m.pose.orientation.w)
        #yaw = euler_from_quaternion(q)[2]
        #NODES.append((m.id, m.pose.position.x, m.pose.position.y, yaw))

#def build_initialpose(x,y,yaw, frame_id="map",
                      #cov_xy=0.25, cov_yaw=(10.0*math.pi/180.0)**2):
    #qx,qy,qz,qw = quaternion_from_euler(0,0,yaw)
    #m = PoseWithCovarianceStamped()
    #m.header.frame_id = frame_id
    #m.header.stamp = rospy.Time.now()
    #m.pose.pose.position.x = x
    #m.pose.pose.position.y = y
    #m.pose.pose.orientation = Quaternion(qx,qy,qz,qw)
    #cov = [0.0]*36; cov[0]=cov[7]=cov_xy; cov[35]=cov_yaw
    #m.pose.covariance = cov
    #return m

#def prefix_without_ext(path):
    #if path.endswith(".posegraph") or path.endswith(".data"):
        #return os.path.splitext(path)[0]
    #return path

#if __name__ == "__main__":
    #rospy.init_node("load_map_and_save_node0_deserialize")

    #posegraph_in = rospy.get_param("~posegraph", "")
    #out_yaml     = rospy.get_param("~out_yaml", "initial_pose.yaml")
    #publish_ip   = bool(rospy.get_param("~publish_initialpose", False))
    #initpose_topic = rospy.get_param("~initialpose_topic", "/initialpose")
    #graph_topic    = rospy.get_param("~graph_topic", "/slam_toolbox/karto_graph_visualization")
    #slam_node_type = rospy.get_param("~slam_node_type", "sync_slam_toolbox_node")
    #map_frame      = rospy.get_param("~map_frame", "map")
    #odom_frame     = rospy.get_param("~odom_frame", "odom")
    #base_frame     = rospy.get_param("~base_frame", "base_link")
    #timeout_s      = float(rospy.get_param("~timeout_s", 60.0))

    #if not posegraph_in:
        #rospy.logerr("~posegraph is required.")
        #sys.exit(1)
    #prefix = prefix_without_ext(posegraph_in)
    #if not (os.path.isfile(prefix + ".posegraph") and os.path.isfile(prefix + ".data")):
        #rospy.logerr("Could not find both files: %s.posegraph and %s.data", prefix, prefix)
        #sys.exit(1)

    #print("prefix: %s"%prefix)
    #Start slam_toolbox (offline / localization)
    #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)
    #launcher = roslaunch.scriptapi.ROSLaunch()
    #launcher.parent = roslaunch.parent.ROSLaunchParent(uuid, [])
    #launcher.start()

    #ns = "/slam_toolbox"
    #rospy.set_param(ns + "/localization", True)
    #rospy.set_param(ns + "/map_frame",  map_frame)
    #rospy.set_param(ns + "/odom_frame", odom_frame)
    #rospy.set_param(ns + "/base_frame", base_frame)
    #rospy.set_param(ns + "/enable_interactive_mode", False)

    #node = roslaunch.core.Node("slam_toolbox", slam_node_type, name="slam_toolbox", output="screen")
    #proc = launcher.launch(node)
    #if not proc.is_alive():
        #rospy.logerr("Failed to start slam_toolbox.")
        #launcher.stop(); sys.exit(1)

    #Call /slam_toolbox/deserialize_map with the prefix (no extension)
    #srv_name = ns + "/deserialize_map"
    #rospy.loginfo("Waiting for %s ...", srv_name)
    #rospy.wait_for_service(srv_name, timeout=30.0)
    #cli = rospy.ServiceProxy(srv_name, DeserializePoseGraph)
    #req = DeserializePoseGraphRequest()
    #req.filename = prefix            # <-- no extension
    #req.match_type = 2               # 2 = none/exact (we just want to load)
    #req.initial_pose.x = 0.0; req.initial_pose.y = 0.0; req.initial_pose.theta = 0.0
    #cli(req)
    #rospy.loginfo("Deserialized map from %s", prefix)

    #Now wait for graph markers
    #sub = rospy.Subscriber(graph_topic, MarkerArray, marker_cb, queue_size=1)
    #t0 = time.time()
    #rate = rospy.Rate(50)
    #while not rospy.is_shutdown() and (time.time() - t0) < timeout_s and not NODES:
        #rate.sleep()

    #if not NODES:
        #rospy.logerr("No graph markers on %s within %.1fs. Check topic/namespace & file path.",
                     #graph_topic, timeout_s)
        #proc.stop(); launcher.stop(); sys.exit(1)

    #nid, x, y, yaw = sorted(NODES, key=lambda t: t[0])[0]
    #data = {"frame_id": map_frame, "x": float(x), "y": float(y), "yaw": float(yaw)}
    #with open(out_yaml, "w") as f:
        #yaml.safe_dump(data, f, default_flow_style=False)
    #rospy.loginfo("Saved Node%d as start pose -> %s (x=%.3f, y=%.3f, yaw=%.3f)",
                  #nid, out_yaml, x, y, yaw)

    #if publish_ip:
        #pub = rospy.Publisher(initpose_topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
        #for _ in range(50):
            #if pub.get_num_connections() > 0: break
            #rospy.sleep(0.1)
        #pub.publish(build_initialpose(x,y,yaw, frame_id=map_frame))
        #rospy.loginfo("Published %s", initpose_topic)

    #proc.stop(); launcher.stop()
