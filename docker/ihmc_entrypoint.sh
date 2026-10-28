#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/elevation_ws/install/setup.bash
exec "$@"
