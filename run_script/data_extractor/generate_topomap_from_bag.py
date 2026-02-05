#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import subprocess
from pathlib import Path
from typing import Optional
import yaml
import numpy as np

def error(msg: str, code: int = 1) -> None:
    print(f"\033[31m[ERROR] {msg}\033[0m", file=sys.stderr)
    raise SystemExit(code)


def run(cmd, *, cwd=None, shell=False, env=None) -> None:
    """Run a command and fail fast (like set -e)."""
    print(f"[CMD] {cmd if isinstance(cmd, str) else ' '.join(cmd)}")
    try:
        subprocess.run(cmd, cwd=cwd, shell=shell, env=env, check=True)
    except subprocess.CalledProcessError as e:
        error(f"Command failed (exit={e.returncode}): {cmd}")


def read_yaml_value(path: Path, key: str) -> str:
    """
    Minimal YAML key parser matching the bash grep/sed/awk behavior:
    - finds first line like: key: value
    - strips comments after '#'
    - strips whitespace
    """
    if not path.is_file():
        error(f"YAML config not found: {path}")

    for line in path.read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if not s.startswith(f"{key}:"):
            continue
        # remove inline comments
        before_hash = line.split("#", 1)[0]
        # split at first colon
        parts = before_hash.split(":", 1)
        if len(parts) != 2:
            continue
        value = parts[1].strip()
        # drop surrounding quotes if present
        if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
            value = value[1:-1].strip()
        return value

    return ""


def bash_lc(command: str, *, cwd: Optional[Path] = None) -> None:
    """
    Execute a command inside a login-ish bash (-lc), so `source` and `conda activate` work.
    """
    run(["bash", "-lc", command], cwd=str(cwd) if cwd else None)



# --- Copy extracted data files ---
def copy_required(src: Path, dst_dir: Path) -> None:
    if not src.is_file():
        error(f"Missing required file: {src}")
    print(f"[COPY] {src} -> {dst_dir}/")
    shutil.copy2(src, dst_dir / src.name)


def create_synced_topomap(
    input_dir,
    topomap_dir,
    base_dir,
    config_path=None,
    ts=10,
):
    """
    Port of create_synced_topomap.py (without argparse/CLI).
    - input_dir: extracted data dir (contains 'synced/' folder)
    - topomap_dir: destination topomap folder
    - base_dir: BASE_DIR used to locate defaults.yaml (your repo base)
    - config_path: path to depth_nav.yaml (optional; default uses base_dir/config/depth_nav.yaml)
    - ts: subgoal spacing step (create_synced_topomap.py hard-coded 10)
    """
    input_dir = Path(input_dir).expanduser().resolve()
    topomap_dir = Path(topomap_dir).expanduser().resolve()
    base_dir = Path(base_dir).expanduser().resolve()

    if config_path is None:
        config_path = base_dir / "config" / "depth_nav.yaml"
    else:
        config_path = Path(config_path).expanduser().resolve()

    defaults_path = base_dir / "config" / "defaults.yaml"

    if not defaults_path.is_file():
        raise FileNotFoundError(f"defaults.yaml not found: {defaults_path}")
    if not config_path.is_file():
        raise FileNotFoundError(f"config not found: {config_path}")

    # load configs
    with defaults_path.open("r") as f:
        default_config = yaml.safe_load(f) or {}
    with config_path.open("r") as f:
        user_config = yaml.safe_load(f) or {}

    config = dict(default_config)
    config.update(user_config)

    sync_data_dir = input_dir / "synced"
    if not sync_data_dir.is_dir():
        raise FileNotFoundError(f"synced dir not found: {sync_data_dir}")

    topomap_dir.mkdir(parents=True, exist_ok=True)

    # load synced arrays
    sync_odom = np.loadtxt(str(sync_data_dir / "sync_odom.txt"))
    sync_tf_m2b = np.loadtxt(str(sync_data_dir / "sync_tf_m2b.txt"))
    sync_tf_m2o = np.loadtxt(str(sync_data_dir / "sync_tf_m2o.txt"))

    num_rgb = len(sync_odom)  # == num_odom
    topo_odom = sync_odom[0:num_rgb:ts]
    topo_m2b = sync_tf_m2b[0:num_rgb:ts]
    topo_m2o = sync_tf_m2o[0:num_rgb:ts]

    topo_rgb_idx = list(range(0, num_rgb, ts))
    rgb_filenames = [f"rgb{idx:05d}.png" for idx in topo_rgb_idx]
    depth_filenames = [f"depth{idx:05d}.png" for idx in topo_rgb_idx]

    # copy images
    def copy_list(fnames):
        for fname in fnames:
            src = sync_data_dir / fname
            dst = topomap_dir / fname
            if src.exists():
                shutil.copy2(str(src), str(dst))
            else:
                print(f"\033[91mFile not found: {src}\033[0m")

    copy_list(rgb_filenames)
    copy_list(depth_filenames)

    # save topo files
    np.savetxt(str(topomap_dir / "topo_odom.txt"), topo_odom)
    np.savetxt(str(topomap_dir / "topo_tf_m2b.txt"), topo_m2b)
    np.savetxt(str(topomap_dir / "topo_tf_m2o.txt"), topo_m2o)

    num_nodes = len(topo_m2b)
    print("Finished creating %d topo nodes" % num_nodes)

def main() -> int:
    # --- Variables (mirror bash) ---
    # NOTE: In bash you used "~" inside quotes which doesn't expand. Fix here via expanduser().
    
    AINAV_PROJ_DIR = Path("~/python_ws/viznav/depth-nav").expanduser().resolve()
    #AINAV_PROJ_DIR = Path("/home/python_ws/viznav/depth-nav/deployment").expanduser()
    CATKIN_WS = Path(os.environ.get("CATKIN_WS", str(Path.home() / "catkin_ws"))).expanduser()

    NAVDATA_EXTRACTOR_DIR = CATKIN_WS / "src/navdata_collector/run_script/data_extractor"
    EXTRACTOR_SCRIPT = NAVDATA_EXTRACTOR_DIR / "script_extract_bags.py"
    NAV_CFG = CATKIN_WS / "src/navdata_collector/param/navdata_collector.yaml"

    PROJECT_DIR = AINAV_PROJ_DIR

    print(f"AI-nav project dir     : {PROJECT_DIR}")

    # --- Read YAML values ---
    SRC_BAG_DIR = read_yaml_value(NAV_CFG, "inpath")
    TOPOMAP_DIR = read_yaml_value(NAV_CFG, "topomap_path")
    BASE_EXTRACT_DIR = read_yaml_value(NAV_CFG, "outpath")

    print(f"src bag dir            : {SRC_BAG_DIR}")
    print(f"Topomap dir            : {TOPOMAP_DIR}")
    print(f"Base Ext dir(processed): {BASE_EXTRACT_DIR}")

    if not TOPOMAP_DIR:
        error(f"'topomap_path' is missing/empty in {NAV_CFG}")
    if not BASE_EXTRACT_DIR:
        error(f"'outpath' is missing/empty in {NAV_CFG}")

    SRC_BAG_DIR_P = Path(SRC_BAG_DIR).expanduser()
    TOPOMAP_DIR_P = Path(TOPOMAP_DIR).expanduser()
    BASE_EXTRACT_DIR_P = Path(BASE_EXTRACT_DIR).expanduser()

    # --- Derive BAG_ID from SRC_BAG_DIR (drop trailing slash, take basename) ---
    val = str(SRC_BAG_DIR_P).rstrip("/")
    BAG_ID = Path(val).name  # e.g. T1-2025-09-17-17-44
    BASE_OUT_DIR = BASE_EXTRACT_DIR_P / BAG_ID

    # --- Ensure outpath exists ---
    BASE_OUT_DIR.mkdir(parents=True, exist_ok=True)

    # --- Validate inpath ---
    if not SRC_BAG_DIR_P.exists() or not SRC_BAG_DIR_P.is_dir():
        error(f"inpath dir not found: {SRC_BAG_DIR_P}")

    # --- Find exactly one bag file ---
    bag_files = sorted(SRC_BAG_DIR_P.glob("*.bag"))
    if len(bag_files) != 1:
        error(f"Expected exactly ONE .bag file in {SRC_BAG_DIR_P}, found {len(bag_files)}")
    BAG_FILE = bag_files[0]
    BAG_NAME = BAG_FILE.stem

    # (Original bash created EXTRACTED_DATA_DIR then later overrides it with latest bag_* folder.
    # We'll keep both behaviors: create expected dir, then later discover latest bag_* if present.)
    expected_extracted_dir = BASE_OUT_DIR / BAG_NAME
    expected_extracted_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Using output dir: {expected_extracted_dir}")
    print("[INFO] Bag inputs:")
    for bf in bag_files:
        print(f"  - {bf}")

    # --- Determine latest bag_* under BASE_OUT_DIR (matches bash behavior) ---
    kids = sorted(BASE_OUT_DIR.glob("bag_*"))
    if not kids:
        error(f"no bag_* under {BASE_OUT_DIR}")
    # sort by mtime desc
    kids.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    EXTRACTED_DATA_DIR = kids[0]

    print(f"Ext data dir           : {EXTRACTED_DATA_DIR}")
    print(f"Nav Config file        : {NAV_CFG}")
    print(f"NAVDATA extr directory : {NAVDATA_EXTRACTOR_DIR}")
    print(f"BAG ID                 : {BAG_ID}")

    if not EXTRACTOR_SCRIPT.is_file():
        error(f"Extractor script not found: {EXTRACTOR_SCRIPT}")
    if not NAV_CFG.is_file():
        error(f"Navdata config not found: {NAV_CFG}")

    # --- check for nav_data file ---
    nav_file = SRC_BAG_DIR_P / "nav_data"
    if not nav_file.is_file():
        error(f"Required file 'nav_data' not found in {SRC_BAG_DIR_P}\n"
              f"Make sure to point to the bag file for a topomap")

    print(f"Found nav_data file    : {nav_file}")

    # --- Run extraction inside ROS + conda environment ---
    # Equivalent to:
    # source ~/catkin_ws/install/setup.bash
    # source "$(conda info --base)/etc/profile.d/conda.sh"
    # conda activate navdata
    # cd "$NAVDATA_EXTRACTOR_DIR"
    # python script_extract_bags.py "../../param/navdata_collector.yaml"
    extractor_cmd = (
        f"source {CATKIN_WS}/install/setup.bash && "
        f"source \"$(conda info --base)/etc/profile.d/conda.sh\" && "
        f"conda activate navdata && "
        f"cd \"{NAVDATA_EXTRACTOR_DIR}\" && "
        f"python \"{EXTRACTOR_SCRIPT}\" \"../../param/navdata_collector.yaml\""
    )
    print(f"executing {EXTRACTOR_SCRIPT}")
    bash_lc(extractor_cmd)

    # --- Make directory for topomap ---
    if TOPOMAP_DIR_P.exists():
        error(f"Topomap dir already exists. Remove {TOPOMAP_DIR_P} before creating a new one")
    TOPOMAP_DIR_P.mkdir(parents=True, exist_ok=True)

    copy_required(EXTRACTED_DATA_DIR / "map.data", TOPOMAP_DIR_P)
    copy_required(EXTRACTED_DATA_DIR / "map.posegraph", TOPOMAP_DIR_P)
    copy_required(EXTRACTED_DATA_DIR / "slam_map.png", TOPOMAP_DIR_P)
    copy_required(EXTRACTED_DATA_DIR / "slam_map.yaml", TOPOMAP_DIR_P)

#    assert 0

    print("finished copying topomap, creating slam_poses.txt")

    # --- gen slam_poses ---
    PATH_TO_PGO = TOPOMAP_DIR_P / "map"
    OUT_POSE_TXT = TOPOMAP_DIR_P / "slam_poses.txt"

    # This assumes rosrun is available in current shell; run via bash -lc with ROS sourced.
    dump_cmd = (
        f"source {CATKIN_WS}/install/setup.bash && "
        f"rosrun navdata_collector dump_posegraph \"{PATH_TO_PGO}\" \"{OUT_POSE_TXT}\""
    )
    bash_lc(dump_cmd)

    print("finished decoding PGO file")

    if not OUT_POSE_TXT.is_file():
        error(f"Failed to create {OUT_POSE_TXT}.. Something wrong here..")

    # --- run create_synced_topomap.py ---
    #create_synced = AINAV_PROJ_DIR / "src/create_synced_topomap.py"
    #if not create_synced.is_file():
        #error(f"create_synced_topomap.py not found at: {create_synced}")

    #sync_cmd = (
        #f"cd \"{AINAV_PROJ_DIR / 'src'}\" && "
        #f"python \"{create_synced}\" -i \"{EXTRACTED_DATA_DIR}\" -o \"{TOPOMAP_DIR_P}\""
    #)
    #bash_lc(sync_cmd)

    create_synced_topomap(
        input_dir=EXTRACTED_DATA_DIR,
        topomap_dir=TOPOMAP_DIR_P,
        base_dir=AINAV_PROJ_DIR,
        config_path=AINAV_PROJ_DIR / "config" / "depth_nav.yaml",  # 원래 default와 동일
        ts=10,
    )


    # --- activate vint_deployment (best-effort) ---
    # In bash this just activates and prints Done.
    # Here we just try a quick 'python -c' to verify env exists; otherwise warn.
    env_check_cmd = (
        "if command -v conda >/dev/null 2>&1; then "
        "source \"$(conda info --base)/etc/profile.d/conda.sh\" && "
        "conda activate vint_deployment && "
        "python -c \"import sys; print(sys.executable)\"; "
        "else "
        "echo \"[WARN] conda not on PATH; skipping conda activate. Ensure deps are available.\"; "
        "fi"
    )
    bash_lc(env_check_cmd)

    print(f"[INFO] Done. new topomap created at {TOPOMAP_DIR_P}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
