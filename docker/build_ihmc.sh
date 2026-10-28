#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec docker build --build-arg "BASE_IMAGE=${BASE_IMAGE:-ihmc-reasoning-hydra:jazzy}" \
  -f "$repo/docker/Dockerfile.ihmc" -t "${ELEVATION_IMAGE:-ihmc-elevation-mapping:jazzy}" "$repo" "$@"
