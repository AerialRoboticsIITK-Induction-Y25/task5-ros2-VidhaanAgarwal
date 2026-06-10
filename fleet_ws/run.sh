#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
IMAGE_NAME=drone_fleet_jazzy

echo "Building Multi-Stage Docker Image"
sudo docker build -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${REPO_ROOT}"

echo "Launching Container"
sudo docker run --rm -it \
  --net=host \
  -e ROS_DOMAIN_ID=42 \
  -v "${SCRIPT_DIR}/src:/app_ws/src:ro" \
  "${IMAGE_NAME}"