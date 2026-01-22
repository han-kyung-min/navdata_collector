import os, glob, math
import numpy as np

from os.path import dirname, abspath
BASE_DIR = dirname(dirname(abspath(__file__)))
import sys
sys.path.append(BASE_DIR)
import utils.rigid_motion as rm
import imageio.v2 as imageio
import matplotlib.pyplot as plt
import yaml
from PIL import Image as PILImage


def read_scan_txt(path):
    data = np.loadtxt(path)  # Nx2: angle, range
    if data.ndim == 1:
        data = data[None, :]
    return data[:, 0], data[:, 1]

def bresenham(x0, y0, x1, y1):
    pts = []
    dx, dy = abs(x1-x0), abs(y1-y0)
    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    if dy <= dx:
        err = dx // 2
        while x != x1:
            pts.append((x,y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y1:
            pts.append((x,y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    pts.append((x1,y1))
    return pts

def world_to_grid(x, y, origin_x, origin_y, res):
    gx = int(math.floor((x - origin_x) / res))
    gy = int(math.floor((y - origin_y) / res))
    return gx, gy

def in_bounds(gx, gy, W, H):
    return 0 <= gx < W and 0 <= gy < H

def build_map(scan_dir, poses_xytheta, res=0.05, pad=2.0,
              rmin=0.05, rmax=20.0,
              l_occ=0.85, l_free=-0.40, clip=5.0):

    scan_paths = sorted(glob.glob(os.path.join(scan_dir, "scan*.txt")))
    poses = np.asarray(poses_xytheta, dtype=np.float64)
    assert len(scan_paths) == len(poses), "Need N scans and N poses"

    # 1) auto bounds (transform endpoints roughly)
    all_xy = []
    for k, sp in enumerate(scan_paths):
        ang, rng = read_scan_txt(sp)
        m = np.isfinite(rng) & (rng > rmin) & (rng < rmax)
        ang, rng = ang[m], rng[m]
        if len(rng) == 0:
            continue

        x, y, th = poses[k]
        c, s = math.cos(th), math.sin(th)

        xs = rng * np.cos(ang)
        ys = rng * np.sin(ang)

        xw = c*xs - s*ys + x
        yw = s*xs + c*ys + y

        all_xy.append(np.stack([xw, yw], axis=1))
        all_xy.append(np.array([[x, y]]))  # origin too

    all_xy = np.concatenate(all_xy, axis=0)
    minx, miny = all_xy.min(axis=0) - pad
    maxx, maxy = all_xy.max(axis=0) + pad

    W = int(math.ceil((maxx - minx) / res))
    H = int(math.ceil((maxy - miny) / res))
    origin_x, origin_y = minx, miny

    logodds = np.zeros((H, W), dtype=np.float32)

    # 2) integrate each scan
    for k, sp in enumerate(scan_paths):
        ang, rng = read_scan_txt(sp)
        m = np.isfinite(rng) & (rng > rmin) & (rng < rmax)
        ang, rng = ang[m], rng[m]
        if len(rng) == 0:
            continue

        x, y, th = poses[k]
        c, s = math.cos(th), math.sin(th)

        # sensor origin in world (assuming lidar at base origin)
        gx0, gy0 = world_to_grid(x, y, origin_x, origin_y, res)
        if not in_bounds(gx0, gy0, W, H):
            continue

        xs = rng * np.cos(ang)
        ys = rng * np.sin(ang)

        xw = c*xs - s*ys + x
        yw = s*xs + c*ys + y

        for px, py in zip(xw, yw):
            gx1, gy1 = world_to_grid(px, py, origin_x, origin_y, res)
            if not in_bounds(gx1, gy1, W, H):
                continue

            line = bresenham(gx0, gy0, gx1, gy1)
            if len(line) < 2:
                continue

            # free along the ray (except endpoint)
            for gx, gy in line[:-1]:
                logodds[gy, gx] += l_free

            # occupied at endpoint
            gx, gy = line[-1]
            logodds[gy, gx] += l_occ

        np.clip(logodds, -clip, clip, out=logodds)

    prob = 1.0 / (1.0 + np.exp(-logodds))
    return prob, logodds, (origin_x, origin_y), res


def save_ros_map(prob, origin_xy, res, yaml_path="map.yaml",
                 occupied_thresh=0.65, free_thresh=0.45):
    """
    prob: HxW float in [0,1]
    origin_xy: (origin_x, origin_y) in meters (world coords) for the map's bottom-left
    res: meters/cell
    Creates map.pgm (actually png here for convenience) + map.yaml (ROS map_server style)
    """
    prob = np.asarray(prob, dtype=np.float32)
    H, W = prob.shape

    # Convert prob -> ROS-like occupancy image (0..254) where:
    #   0 = free (white), 100 = occupied (black in PGM convention via negate), 205 unknown
    occ = np.full((H, W), 205, dtype=np.uint8)  # unknown
    occ[prob <= free_thresh] = 0
    occ[prob >= occupied_thresh] = 100

    # map_server expects an image where:
    #   0 = occupied (black), 254/255 = free (white) if negate=0
    # So invert the 0..100 style to 0..255 intensity:
    # We'll produce a grayscale "image" with:
    #   occupied -> 0 (black)
    #   free     -> 254 (white)
    #   unknown  -> 205 (gray)
    img = np.full((H, W), 205, dtype=np.uint8)
    img[occ == 0] = 254
    img[occ == 100] = 0

    # Flip vertically for image coordinates (top-left origin)
    img_to_save = np.flipud(img)

    img_path = yaml_path.replace(".yaml", ".png")  # png works fine for many uses
    PILImage.fromarray(img_to_save, mode="L").save(img_path)

    # ROS YAML
    meta = {
        "image": img_path.split("/")[-1],
        "resolution": float(res),
        "origin": [float(origin_xy[0]), float(origin_xy[1]), 0.0],
        "negate": 0,
        "occupied_thresh": float(occupied_thresh),
        "free_thresh": float(free_thresh),
    }
    with open(yaml_path, "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    print(f"Saved ROS map: {img_path}, {yaml_path}")

def _load_yaml_minimal(yaml_path: str) -> dict:
    """
    Minimal YAML loader for typical ROS map_server yaml files.
    Tries PyYAML; if not available, parses simple 'key: value' lines.
    """
    try:
        import yaml  # PyYAML
        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)
    except Exception:
        meta = {}
        with open(yaml_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line:
                    continue
                k, v = line.split(":", 1)
                k = k.strip()
                v = v.strip()

                # parse lists like [x, y, yaw]
                if v.startswith("[") and v.endswith("]"):
                    items = [t.strip() for t in v[1:-1].split(",")]
                    out = []
                    for it in items:
                        try:
                            out.append(float(it))
                        except ValueError:
                            out.append(it.strip('"').strip("'"))
                    meta[k] = out
                    continue

                # parse numbers/bools/strings
                if v.lower() in ("true", "false"):
                    meta[k] = (v.lower() == "true")
                    continue
                try:
                    if "." in v or "e" in v.lower():
                        meta[k] = float(v)
                    else:
                        meta[k] = int(v)
                    continue
                except ValueError:
                    meta[k] = v.strip('"').strip("'")
        return meta


def load_ros_map(yaml_path: str):
    """
    Load ROS map_server YAML + image and reconstruct:
      - prob: HxW float32 in {0.0, 0.5, 1.0} (free/unknown/occupied)
      - origin: (origin_x, origin_y)
      - res: resolution (m/cell)
      - occ_grid: HxW int8 in {-1, 0, 100} like nav_msgs/OccupancyGrid.data

    Notes:
      - This cannot recover log-odds. ROS map files do not store it.
      - 'prob' is quantized because the source image is quantized.
    """
    meta = _load_yaml_minimal(yaml_path)

    # Required fields (typical ROS map yaml)
    img_rel = meta["image"]
    res = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    origin_xy = (float(origin[0]), float(origin[1]))

    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))  # ROS default often ~0.196

    # Resolve image path relative to yaml directory if needed
    if not os.path.isabs(img_rel):
        img_path = os.path.join(os.path.dirname(yaml_path), img_rel)
    else:
        img_path = img_rel

    # Load grayscale image (0..255)
    img = np.array(PILImage.open(img_path).convert("L"), dtype=np.uint8)

    # Undo the "image coordinate" orientation: images are top-left origin,
    # maps typically treat origin at bottom-left in world.
    img = np.flipud(img)

    # Convert image intensity to occupancy probability p in [0,1]
    # map_server convention (for negate=0):
    #   occ if p > occupied_thresh
    #   free if p < free_thresh
    #
    # map_server computes p from pixel value v approximately:
    #   p = (255 - v) / 255  (for negate=0)
    #   p = v / 255          (for negate=1)
    v = img.astype(np.float32)
    if negate == 0:
        p = (255.0 - v) / 255.0
    else:
        p = v / 255.0

    # Build OccupancyGrid-style values: -1 unknown, 0 free, 100 occupied
    occ_grid = np.full(p.shape, -1, dtype=np.int8)
    occ_grid[p < free_thresh] = 0
    occ_grid[p > occupied_thresh] = 100

    # Also provide a simple prob map compatible with your pipeline:
    # free -> 0.0, unknown -> 0.5, occupied -> 1.0
    prob = np.full(p.shape, 0.5, dtype=np.float32)
    prob[occ_grid == 0] = 0.0
    prob[occ_grid == 100] = 1.0

    return prob, origin_xy, res, occ_grid, meta

def build_costmap(
    prob,                          # HxW float in [0,1]
    logodds=None,                  # optional HxW float (if you have it)
    occ_thresh=0.65,               # prob >= occ_thresh -> obstacle
    free_thresh=0.45,              # prob <= free_thresh -> free
    unknown_band_logodds=0.2,      # if logodds is given: |logodds| < band -> unknown
    inflation_radius_m=0.30,       # meters
    resolution=0.05,               # meters/cell
    lethal_cost=254,               # ROS costmap2d convention
    inflated_cost=253,             # ROS costmap2d convention (used as "inflation layer")
    unknown_cost=255,              # ROS costmap2d convention
    use_gradient=True,             # if True, fill 1..252 costs by distance-to-obstacle
):
    """
    Returns:
      costmap: HxW uint8 with ROS-ish values:
        0 free, 1..252 gradient (optional), 253 inflated, 254 lethal obstacle, 255 unknown
    """
    prob = np.asarray(prob, dtype=np.float32)
    H, W = prob.shape

    # 1) classify base occupancy
    obstacle = prob >= occ_thresh
    free = prob <= free_thresh

    if logodds is not None:
        logodds = np.asarray(logodds, dtype=np.float32)
        unknown = np.abs(logodds) < unknown_band_logodds
    else:
        # fallback unknown as the "uncertain band"
        unknown = (~obstacle) & (~free)

    # initialize costmap
    costmap = np.zeros((H, W), dtype=np.uint8)  # default free=0
    costmap[unknown] = unknown_cost
    costmap[obstacle] = lethal_cost

    # 2) inflation
    r_cells = int(math.ceil(inflation_radius_m / float(resolution)))
    if r_cells <= 0:
        return costmap

    # We'll compute distance-to-obstacle in cells.
    # Try fast distance transform (OpenCV), then SciPy, else fallback dilation.
    dist = None

    # obstacle mask as uint8: 0 free, 1 obstacle
    obs_u8 = obstacle.astype(np.uint8)

    # --- Option A: OpenCV distanceTransform (fast) ---
    try:
        import cv2  # noqa
        # distanceTransform expects 0 for obstacles and non-zero for free
        free_u8 = (1 - obs_u8).astype(np.uint8)
        # returns distance in pixels (cells)
        dist = cv2.distanceTransform(free_u8, distanceType=cv2.DIST_L2, maskSize=5)
    except Exception:
        pass

    # --- Option B: SciPy distance_transform_edt (fast) ---
    if dist is None:
        try:
            from scipy.ndimage import distance_transform_edt  # noqa
            dist = distance_transform_edt(~obstacle).astype(np.float32)  # distance in cells
        except Exception:
            pass

    if dist is not None:
        # cells within inflation radius (excluding lethal cells)
        in_inflation = (dist > 0) & (dist <= r_cells) & (~obstacle)

        if use_gradient:
            # Map distance -> cost 1..252 (higher cost near obstacle)
            # d=1 -> near max, d=r_cells -> near min
            # keep unknown as unknown; do not overwrite unknown
            d = dist[in_inflation]
            # scale to [1,252]
            # cost = round(1 + (252-1) * (1 - (d-1)/(r_cells-1)))
            if r_cells == 1:
                grad = np.full_like(d, 252, dtype=np.float32)
            else:
                grad = 1.0 + (252.0 - 1.0) * (1.0 - (d - 1.0) / (r_cells - 1.0))
            grad = np.clip(np.rint(grad), 1, 252).astype(np.uint8)

            # write gradient only on non-unknown cells
            idx = np.where(in_inflation & (costmap != unknown_cost))
            costmap[idx] = np.maximum(costmap[idx], grad)
        else:
            # mark inflated band as 253
            idx = np.where(in_inflation & (costmap != unknown_cost))
            costmap[idx] = np.maximum(costmap[idx], np.uint8(inflated_cost))

        return costmap

    # --- Option C: No distance transform available -> simple binary dilation (no gradient) ---
    # This is slower but dependency-free; it marks the inflation band as 253.
    # Create a disk kernel and dilate obstacle mask by shifting.
    yy, xx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
    disk = (xx * xx + yy * yy) <= (r_cells * r_cells)

    inflated = obstacle.copy()
    # shift-add dilation
    ky, kx = np.where(disk)
    ky = ky - r_cells
    kx = kx - r_cells
    base = obstacle
    inflated = np.zeros_like(base, dtype=bool)
    for dy, dx in zip(ky, kx):
        y0 = max(0, dy)
        y1 = min(H, H + dy)
        x0 = max(0, dx)
        x1 = min(W, W + dx)
        inflated[y0:y1, x0:x1] |= base[y0 - dy:y1 - dy, x0 - dx:x1 - dx]

    in_inflation = inflated & (~obstacle)
    idx = np.where(in_inflation & (costmap != unknown_cost))
    costmap[idx] = np.maximum(costmap[idx], np.uint8(inflated_cost))

    return costmap


# -----------------------------------------------------------------------------
# ROS-like costmap colorization (RViz-ish)
# -----------------------------------------------------------------------------
def costmap_to_rgb(costmap: np.ndarray) -> np.ndarray:
    """
    Color convention:
      Free (0)            -> Bright gray
      Unknown (255)       -> Dark gray
      Inflation (1..252)  -> Cyan
      Inscribed (253)     -> Red
      Lethal (254)        -> Red
    """
    cm = np.asarray(costmap, dtype=np.uint8)
    H, W = cm.shape
    rgb = np.zeros((H, W, 3), dtype=np.uint8)

    # FREE -> bright gray
    rgb[cm == 0] = (220, 220, 220)

    # UNKNOWN -> dark gray
    rgb[cm == 255] = (120, 120, 120)

    # INFLATION (gradient band) -> cyan
    infl = (cm >= 1) & (cm <= 252)
    rgb[infl] = (0, 255, 255)

    # INSCRIBED + LETHAL -> red
    danger = (cm == 253) | (cm == 254)
    #rgb[danger] = (255, 0, 0)

    return rgb

if __name__ == "__main__":
    base_dir = "/media/data/results/navdata_collector/topomap/processed/T1-2025-09-17-17-44/bag_2025-09-17-17-44-56/synced"
    scan_dir = base_dir
    m2b_pose_file = "%s/sync_tf_m2b.txt"%base_dir
    m2b_pose = np.loadtxt(m2b_pose_file)  # Nx3: x y theta(rad)
    num_poses = len(m2b_pose)
    bHs = rm.xyzrpy_to_htm([0.159, -0.000, 0.166, 3.142, 0., 0.])

    poses = np.zeros([num_poses, 3], dtype=float)
    for ii in range(0, num_poses, 10):
        print(m2b_pose[ii])
        x, y = m2b_pose[ii][4:6]
        qx, qy, qz, qw = m2b_pose[ii][7:]
        mHb = rm.quat_to_htm([qw, qx, qy, qz])
        mHb[0,3] = x
        mHb[1,3] = y
        mHs = np.matmul(mHb, bHs)
        x, y, _, _, _, yaw = rm.htm_to_xyzrpy(mHs)
        poses[ii] = [x,y,yaw]

    # prob, logodds, origin, res = build_map(scan_dir, poses, res=0.05)
    # save_ros_map(prob, origin, res, yaml_path="map.yaml",
    #              occupied_thresh=0.65, free_thresh=0.45)

    prob, origin, res, occ_grid, meta = load_ros_map("map.yaml")

    costmap = build_costmap(prob=prob, logodds=None, resolution=res, inflation_radius_m=0.3)
    rgb = costmap_to_rgb(costmap)

    rgb = np.asarray(rgb, dtype=np.uint8)
    if True: #flipud:
        rgb = np.flipud(rgb)  # make it look like typical x-right, y-up visualization
    plt.imsave("costmap.png", rgb)

    # visualize/save

    # Convert to occupancy image: black=occupied, white=free/unknown (simple threshold)
    occ = (prob > 0.55).astype(np.uint8) * 255
    imageio.imwrite("map.png", np.flipud(occ))
    print("Saved map.png, origin:", origin, "res:", res)