#!/bin/bash
set -euo pipefail
#set -x

AINAV_PROJ_DIR="/home/hankm/python_ws/viznav/depth-nav/deployment"
CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
NAVDATA_EXTRACTOR_DIR="$CATKIN_WS/src/navdata_collector/run_script/data_extractor"
EXTRACTOR_SCRIPT="$NAVDATA_EXTRACTOR_DIR/script_extract_bags.py"
NAV_CFG="$CATKIN_WS/src/navdata_collector/param/navdata_collector.yaml"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$AINAV_PROJ_DIR" #"$(realpath "$SCRIPT_DIR/../..")"

echo "AI-nav project dir     : $PROJECT_DIR"

TOPOMAP_DIR="$(
  grep -E '^[[:space:]]*topomap_path:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"

SRC_BAG_DIR="$(
  grep -E '^[[:space:]]*inpath:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"


EXTRACTED_DATA_DIR="$(
  grep -E '^[[:space:]]*outpath:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"

[ -n "${TOPOMAP_DIR:-}" ] || { echo "[ERROR] 'topomap_name' is missing/empty in $NAV_CFG"; exit 1; }
[ -n "${EXTRACTED_DATA_DIR:-}" ] || { echo "[ERROR] 'out_path' is missing/empty in $NAV_CFG"; exit 1; }

VAL=${SRC_BAG_DIR%/}              # drop trailing slash if any
BAG_ID=${VAL##*/}           # -> T1-2025-09-17-17-44
EXTRACTED_DATA_DIR="${EXTRACTED_DATA_DIR}/${BAG_ID}"

echo "Nav Config file : $NAV_CFG"
echo $NAVDATA_EXTRACTOR_DIR
echo $BAG_ID

echo "TOPOMAP_DIR:  $TOPOMAP_DIR"  
echo "ext data dir: $EXTRACTED_DATA_DIR"


if [[ ! -f "$EXTRACTOR_SCRIPT" ]]; then
    echo "[ERROR] Extractor script not found: $EXTRACTOR_SCRIPT" >&2
    exit 1
fi
if [[ ! -f "$NAV_CFG" ]]; then
    echo "[ERROR] Navdata config not found: $NAV_CFG" >&2
    exit 1
fi

# --- Run extraction inside ROS + conda environment ---
source ~/catkin_ws/install/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate navdata
cd "$NAVDATA_EXTRACTOR_DIR"

echo "executing $EXTRACTOR_SCRIPT"
python $EXTRACTOR_SCRIPT "../../param/navdata_collector.yaml"

# 1. Make directory

if [[ -e "$TOPOMAP_DIR" ]]; then
  echo "[ERROR] Topomap dir already exists. I am not overwritting this folder: $TOPOMAP_DIR" >&2
  exit 1
fi

mkdir -p "$TOPOMAP_DIR"

exit 0

# 2. Copy extracted data
# if compgen -G "${EXTRACTED_DATA_DIR}/bag_*/*" > /dev/null; then
#   cp -a -- "${EXTRACTED_DATA_DIR}"/bag_*/* "$TOPOMAP_DIR"/
# fi


echo "finished copying topomap, creating slam_poses.txt"
# 3. gen slam_poses
PATH_TO_PGO="$TOPOMAP_DIR/map"

OUT_POSE_TXT="$TOPOMAP_DIR/slam_poses.txt"
rosrun navdata_collector dump_posegraph $PATH_TO_PGO $OUT_POSE_TXT

echo "finished decoding PGO file"

cd "${AINAV_PROJ_DIR}/src"


python create_synced_topomap.py -i "$TOPOMAP_DIR"

if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate vint_deployment
else
  echo "[WARN] conda not on PATH; skipping conda activate. Ensure deps are available."
fi
echo "[INFO] Done. new topomap created at $TOPOMAP_DIR"

# extract slam_poses.txt

