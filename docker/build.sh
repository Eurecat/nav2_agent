#!/bin/bash
set -euo pipefail

USER_ID=$(id -u)
GROUP_ID=$(id -g)

NO_CACHE=""
if [[ "${1:-}" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
fi

echo "Building nav2_agent:jazzy image for user $USER_ID..."

DOCKER_BUILDKIT=1 docker build -t nav2_agent:jazzy \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    $NO_CACHE .

echo "Built nav2_agent:jazzy. You can now run the container with: docker compose up -d"