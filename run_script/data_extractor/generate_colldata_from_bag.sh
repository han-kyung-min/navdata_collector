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


SRC_BAG_DIR="$(
  grep -E '^[[:space:]]*inpath:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"

EXTRACTED_BAG_DIR="$(
  grep -E '^[[:space:]]*outpath:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"

[ -n "${SRC_BAG_DIR:-}" ] || { echo "[ERROR] 'inpath' is missing/empty in $NAV_CFG"; exit 1; }
[ -n "${EXTRACTED_BAG_DIR:-}" ] || { echo "[ERROR] 'outpath' is missing/empty in $NAV_CFG"; exit 1; }

VAL=${SRC_BAG_DIR%/}              # drop trailing slash if any
BAG_ID=${VAL##*/}           # -> T1-2025-09-17-17-44
EXTRACTED_DATA_DIR="${EXTRACTED_BAG_DIR}/${BAG_ID}"
#[ -d "$BASE_OUT_DIR" ] || { echo "[ERROR] not found: $BASE_OUT_DIR"; exit 1; }

#shopt -s nullglob
#kids=( "$BASE_OUT_DIR"/bag_* )
# No candidates?
#[ ${#kids[@]} -gt 0 ] || { echo "[ERROR] no bag_* under $BASE_OUT_DIR"; exit 1; }
#EXTRACTED_DATA_DIR="$(ls -1dt "${kids[@]}" | head -n 1)"

echo "Nav Config file : $NAV_CFG"
echo $NAVDATA_EXTRACTOR_DIR
echo $BAG_ID
echo "ext data dir: $EXTRACTED_DATA_DIR"


if [[ ! -f "$EXTRACTOR_SCRIPT" ]]; then
    echo "[ERROR] Extractor script not found: $EXTRACTOR_SCRIPT" >&2
    exit 1
fi
if [[ ! -f "$NAV_CFG" ]]; then
    echo "[ERROR] Navdata config not found: $NAV_CFG" >&2
    exit 1
fi

# --- check for coll_data file ---
nav_file="$SRC_BAG_DIR/coll_data"
if [ ! -f "$nav_file" ]; then
  echo "[ERROR] Required file 'coll_data' not found in $SRC_BAG_DIR"
  echo "I can only process collision data !!!"
  exit 1
fi

# --- Run extraction inside ROS + conda environment ---
source ~/catkin_ws/install/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate navdata
cd "$NAVDATA_EXTRACTOR_DIR"

echo "executing $EXTRACTOR_SCRIPT"
python $EXTRACTOR_SCRIPT "../../param/navdata_collector.yaml"


# 2. Copy extracted data
# if compgen -G "${EXTRACTED_DATA_DIR}/bag_*/*" > /dev/null; then
#   cp -a -- "${EXTRACTED_DATA_DIR}"/bag_*/* "$TOPOMAP_DIR"/
# fi

echo "[INFO] Done. coll data created at $EXTRACTED_DATA_DIR"

# extract slam_poses.txt

