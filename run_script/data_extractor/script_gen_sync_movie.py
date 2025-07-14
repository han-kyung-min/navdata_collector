import os
import yaml
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib import gridspec
from scipy.spatial.transform import Rotation as R
import glob
import rigid_motion as rm  # <-- your module with transforms
from tqdm import tqdm

def read_yaml(yaml_path):
    """Reads a YAML file and returns dict."""
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


def quat_to_eul(q):
    """Converts quaternion [qx, qy, qz, qw] to euler angles [yaw, pitch, roll]."""
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    return r.as_euler('zyx', degrees=False)


if __name__ == "__main__":
    # Paths
    base_dir = os.path.abspath(os.path.join(os.getcwd(), "../../"))
    config_file = os.path.join(base_dir, "param", "navdata_collector.yaml")
    config = read_yaml(config_file)

    inpath = config['navdata_extractor']['inpath']
    outpath = config['navdata_extractor']['outpath']
    tmppath = inpath.split('/')[:-1]
    base_sync_metadata_dir = os.path.join('/',*tmppath)

    # navtime_id
    navtime_id = os.path.basename(inpath)
    extraction_dir = os.path.join(outpath, navtime_id)
    pattern = os.path.join(extraction_dir, "bag_*")
    all_matches = glob.glob(pattern)
    processed_bags_dir = [p for p in all_matches if os.path.isdir(p)]

    num_bags = len(processed_bags_dir)
    # Loop over selected indices (MATLAB was 2,3,5,6 => indices 1,2,4,5 in Python 0-based)
    for tmpidx in range(0, num_bags):
        bag_id = processed_bags_dir[tmpidx].split('/')[-1]
        sync_metadata_dir = os.path.join(extraction_dir, bag_id, "synced")

        odom_path = os.path.join(sync_metadata_dir, "sync_odom.txt")
        odom = np.loadtxt(odom_path)

        num_data = odom.shape[0]
        px_min, px_max = np.min(odom[:, 4]), np.max(odom[:, 4])
        py_min, py_max = np.min(odom[:, 5]), np.max(odom[:, 5])

        # Init pose
        px0, py0 = odom[0, 4], odom[0, 5]
        orient0 = quat_to_eul(odom[0, 7:11])
        theta0 = orient0[0]
        hx0, hy0 = np.cos(theta0), np.sin(theta0)

        xy = np.zeros((num_data, 2))
        euc_dist = np.zeros(num_data)
        ang_dist = np.zeros(num_data)

        # Prepare video
        videofilename = f"%s/%s.mp4"%(extraction_dir,bag_id)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(videofilename, fourcc, 40, (1024, 1024))

        fig = plt.figure(figsize=(10, 10))
        gs = gridspec.GridSpec(2, 2)

        # Initial transform
        pose7 = odom[0, 4:11]
        orient = quat_to_eul(pose7[3:])
        theta = orient[0]
        wHr_prev = rm.xyzrpy_to_htm([pose7[0], pose7[1], 0, 0, 0, theta])

        for idx in tqdm(range(num_data - 1), desc=f"Processing video of {bag_id}"):
            rgb_path = os.path.join(sync_metadata_dir, f"rgb{idx:05d}.png")
            depth_path = os.path.join(sync_metadata_dir, f"depth{idx:05d}.png")

            if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
                print(f"Skipping frame {idx} due to missing image files.")
                continue

            rgbimg = cv2.cvtColor(cv2.imread(rgb_path), cv2.COLOR_BGR2RGB)
            depthimg = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            depth_enhanced = cv2.normalize(depthimg.astype(np.float32), None, 0, 1, cv2.NORM_MINMAX)

            pose7 = odom[idx, 4:11]
            orient = quat_to_eul(pose7[3:])
            theta = orient[0]
            px, py = pose7[0], pose7[1]
            xy[idx, :] = [px, py]

            fig.clf()

            # RGB
            ax1 = fig.add_subplot(gs[0, 0])
            ax1.imshow(rgbimg)
            ax1.set_title("RGB")
            ax1.axis("off")

            # Depth raw
            ax2 = fig.add_subplot(gs[0, 1])
            ax2.imshow(depthimg / 255.0, cmap='gray')
            ax2.set_title("Depth raw")
            ax2.axis("off")

            # Depth enhanced
            ax3 = fig.add_subplot(gs[1, 0])
            ax3.imshow(depth_enhanced, cmap='gray')
            ax3.set_title("Depth enhanced")
            ax3.axis("off")

            # Trajectory
            ax4 = fig.add_subplot(gs[1, 1])
            ax4.plot(xy[:idx + 1, 0], xy[:idx + 1, 1], 'r.')
            ax4.plot(px, py, 'oc', markersize=10)
            ax4.quiver(px0, py0, hx0 * 2, hy0 * 2, color='g', scale=1, scale_units='xy')
            ax4.quiver(px, py, np.cos(theta), np.sin(theta), color='b', scale=1, scale_units='xy')
            ax4.set_xlim(px_min - 11, px_max + 11)
            ax4.set_ylim(py_min - 1, py_max + 1)
            ax4.set_aspect('equal')
            ax4.grid(True)
            ax4.set_title("Odom pose and traj")

            plt.tight_layout()
            fig.canvas.draw()

            # Convert figure to image
            img_rgb = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            img_rgb = img_rgb.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            img_bgr = cv2.resize(img_bgr, (1024, 1024))

            video_writer.write(img_bgr)

            # Compute relative transform
            rol, pit, yaw = orient
            wHr = rm.xyzrpy_to_htm([px, py, 0, rol, pit, yaw])
            if idx > 0:
                r0Hr1 = np.linalg.inv(wHr_prev) @ wHr
                dx, dy, dz, droll, dpit, dyaw = rm.htm_to_xyzrpy(r0Hr1)
                euc_dist[idx] = np.sqrt(dx ** 2 + dy ** 2)
                ang_dist[idx] = dyaw
            wHr_prev = wHr

        video_writer.release()
        print(f"Saved video: {videofilename}")
