#!/usr/bin/env bash
###############################################################################
# build_and_run.sh — Build the Docker image and start the container
#
# Usage:
#   ./docker/build_and_run.sh          # Build + run interactive shell
#   ./docker/build_and_run.sh build    # Build only
#   ./docker/build_and_run.sh run      # Run only (image must exist)
###############################################################################
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

IMAGE_NAME="px4-swarm-sitl"
CONTAINER_NAME="px4_swarm_dev"

# Allow X11 connections from Docker
xhost +local:docker 2>/dev/null || true

build_image() {
    echo "══════════════════════════════════════════════════════════════"
    echo "  Building PX4 SITL Docker image (this may take 15-30 min)"
    echo "══════════════════════════════════════════════════════════════"
    docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
    echo "✅ Docker image built successfully"
}

run_container() {
    echo "══════════════════════════════════════════════════════════════"
    echo "  Starting PX4 SITL container"
    echo "══════════════════════════════════════════════════════════════"

    # Remove any existing container with the same name
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --privileged \
        --gpus all \
        --network host \
        --ipc host \
        -e DISPLAY="${DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
        -e XAUTHORITY=/tmp/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro" \
        -v "$PROJECT_DIR:/home/px4user/uav_swarm:rw" \
        "$IMAGE_NAME"
}

case "${1:-all}" in
    build) build_image ;;
    run)   run_container ;;
    all)   build_image && run_container ;;
    *)
        echo "Usage: $0 [build|run|all]"
        exit 1
        ;;
esac
