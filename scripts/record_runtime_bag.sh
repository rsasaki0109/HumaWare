#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROBOT_ID="${ROBOT_ID:-mock_001}"
OUTPUT_DIR="${1:-}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ -f "install/setup.bash" ]]; then
  source install/setup.bash
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  OUTPUT_DIR="artifacts/bags/${ROBOT_ID}_${timestamp}"
fi

mkdir -p "$(dirname "${OUTPUT_DIR}")"

ros2 launch humaware_bag_profiles record_runtime.launch.py \
  robot_id:="${ROBOT_ID}" \
  output_dir:="${OUTPUT_DIR}"
