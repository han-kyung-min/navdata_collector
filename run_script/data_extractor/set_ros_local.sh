#!/usr/bin/env bash

# =========================================
# ROS Local Environment Setup
# =========================================

# Stop on error
set -e

# Optional: unset conflicting hostname (safety)
unset ROS_HOSTNAME
# Set ROS master and local IP
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

# Print status
echo "[INFO] ROS local environment configured"
echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] ROS_IP=$ROS_IP"
