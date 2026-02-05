#!/bin/bash
set -euo pipefail
#set -x

RED="\033[31m"
NC="\033[0m"   # No Color

error() {
    echo -e "\033[31m[ERROR] $*\033[0m" >&2
    exit 1
}

AINAV_PROJ_DIR="~/python_ws/viznav/depth-nav/deployment"
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

echo "src bag dir            : $SRC_BAG_DIR"

TOPOMAP_DIR="$(
  grep -E '^[[:space:]]*topomap_path:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"

echo "Topomap dir            : $TOPOMAP_DIR"


BASE_EXTRACT_DIR="$(
  grep -E '^[[:space:]]*outpath:' "$NAV_CFG" \
    | sed 's/#.*//' \
    | awk -F': *' '{print $2}' \
    | xargs
)"


echo "Base Ext dir(processed): $BASE_EXTRACT_DIR"

[ -n "${TOPOMAP_DIR:-}" ] || error "'topomap_name' is missing/empty in $NAV_CFG"
[ -n "${BASE_EXTRACT_DIR:-}" ] || error "'out_path' is missing/empty in $NAV_CFG"

VAL=${SRC_BAG_DIR%/}              # drop trailing slash if any
BAG_ID=${VAL##*/}           # -> T1-2025-09-17-17-44
BASE_OUT_DIR="${BASE_EXTRACT_DIR}/${BAG_ID}"


# --- Ensure outpath exists ---
mkdir -p -- "$BASE_OUT_DIR" || error "failed to create outpath: $BASE_OUT_DIR"

# --- Define inpath from YAML (SRC_BAG_DIR) ---
INPATH_DIR="$SRC_BAG_DIR"
[[ -n "${INPATH_DIR:-}" ]] || error "inpath is empty in $NAV_CFG"
[[ -d "$INPATH_DIR" ]] || error "inpath dir not found: $INPATH_DIR"

# --- Find bag files in inpath ---
shopt -s nullglob
bag_files=( "$INPATH_DIR"/*.bag )
shopt -u nullglob

(( ${#bag_files[@]} == 1 )) || error "Expected exactly ONE .bag file in $INPATH_DIR, found ${#bag_files[@]}"
BAG_FILE="${bag_files[0]}"
# --- Create a NEW timestamped output folder under BASE_OUT_DIR ---
BAG_NAME="$(basename "$BAG_FILE" .bag)"   # 
EXTRACTED_DATA_DIR="$BASE_OUT_DIR/$BAG_NAME"
mkdir -p -- "$EXTRACTED_DATA_DIR" || error "failed to create: $EXTRACTED_DATA_DIR"

echo "[INFO] Using output dir: $EXTRACTED_DATA_DIR"
echo "[INFO] Bag inputs:"
printf '  - %s\n' "${bag_files[@]}"

shopt -s nullglob
kids=( "$BASE_OUT_DIR"/bag_* )
# No candidates?
[ ${#kids[@]} -gt 0 ] || { error "no bag_* under $BASE_OUT_DIR"; exit 1; }
EXTRACTED_DATA_DIR="$(ls -1dt "${kids[@]}" | head -n 1)"

echo "Ext data dir           : $EXTRACTED_DATA_DIR"
echo "Nav Config file        : $NAV_CFG"
echo "NAVDATA extr directory : $NAVDATA_EXTRACTOR_DIR"
echo "BAG ID                 : $BAG_ID"

if [[ ! -f "$EXTRACTOR_SCRIPT" ]]; then
    error "Extractor script not found: $EXTRACTOR_SCRIPT" >&2
fi
if [[ ! -f "$NAV_CFG" ]]; then
    error "Navdata config not found: $NAV_CFG" >&2
fi

# --- check for nav_data file ---
nav_file="$SRC_BAG_DIR/nav_data"
if [ ! -f "$nav_file" ]; then
  error "[ERROR] Required file 'nav_data' not found in $SRC_BAG_DIR"
  error "Make sure to point to the bag file for a topomap"
fi

echo "Found nav_data file    : $nav_file"

# --- Run extraction inside ROS + conda environment ---
source ~/catkin_ws/install/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate navdata
cd "$NAVDATA_EXTRACTOR_DIR"

echo "executing $EXTRACTOR_SCRIPT"
python $EXTRACTOR_SCRIPT "../../param/navdata_collector.yaml"

# 1. Make directory

if [[ -e "$TOPOMAP_DIR" ]]; then
  error "Topomap dir already exists. Remove $TOPOMAP_DIR before creating a new one" >&2
  exit 1
fi

mkdir -p "$TOPOMAP_DIR"

#2. Copy extracted data
# shopt -s nullglob
# map_files=("${EXTRACTED_DATA_DIR}"/map*)
# 
# if (( ${#map_files[@]} > 0 )); then
#     cp -a -- "${map_files[@]}" "$TOPOMAP_DIR"/
# else
#     error "No map files found in ${EXTRACTED_DATA_DIR}"
# fi
cp -av -- \
  "${EXTRACTED_DATA_DIR}/map.data" \
  "${EXTRACTED_DATA_DIR}/map.posegraph" \
  "$TOPOMAP_DIR"/

cp -av -- \
  "${EXTRACTED_DATA_DIR}/slam_map.png" \
  "${EXTRACTED_DATA_DIR}/slam_map.yaml" \
  "$TOPOMAP_DIR"/

echo "finished copying topomap, creating slam_poses.txt"
# 3. gen slam_poses
PATH_TO_PGO="$TOPOMAP_DIR/map"
OUT_POSE_TXT="$TOPOMAP_DIR/slam_poses.txt"

rosrun navdata_collector dump_posegraph $PATH_TO_PGO $OUT_POSE_TXT

echo "finished decoding PGO file"

if [ ! -f "$nav_file" ]; then
  error "Failed to create $OUT_POSE_TXT.. Something wrong here.." >&2
fi


cd "${AINAV_PROJ_DIR}/src"

python create_synced_topomap.py -i "$EXTRACTED_DATA_DIR" -o "$TOPOMAP_DIR"

if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate vint_deployment
else
  echo "[WARN] conda not on PATH; skipping conda activate. Ensure deps are available."
fi
echo "[INFO] Done. new topomap created at $TOPOMAP_DIR"

# extract slam_poses.txt

