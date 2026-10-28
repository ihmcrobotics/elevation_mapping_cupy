#!/usr/bin/env bash
set -euo pipefail
# Default to a separate DDS domain for the synthetic demonstration.
# Set ROS_DOMAIN_ID to the robot's domain when supplying a robot command/config.
args=(--rm --init --gpus all --network host --ipc host
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-173}"
      -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}")
if [[ -n "${ELEVATION_CONFIG_DIR:-}" ]]; then
  args+=(-v "${ELEVATION_CONFIG_DIR}:/config:ro")
fi
exec docker run "${args[@]}" "${ELEVATION_IMAGE:-ihmc-elevation-mapping:jazzy}" "$@"
