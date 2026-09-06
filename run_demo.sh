#!/usr/bin/env bash
###############################################################################
# run_demo.sh — Top-level entry point for PUSHPAK UAV-X swarm simulation
#
# Usage:
#   ./run_demo.sh --scenario scenarios/scenario_basic.yaml --num-uavs 6
#   ./run_demo.sh --scenario scenarios/scenario_basic.yaml --num-uavs 6 --headless
#   ./run_demo.sh --scenario scenarios/scenario_basic.yaml --num-uavs 6 --out logs/run_001.jsonl
###############################################################################
set -euo pipefail

# Defaults
SCENARIO="scenarios/scenario_basic.yaml"
NUM_UAVS=6
HEADLESS="false"
OUT_PATH="logs/run.jsonl"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --scenario)    SCENARIO="$2"; shift 2 ;;
        --num-uavs)    NUM_UAVS="$2"; shift 2 ;;
        --headless)    HEADLESS="true"; shift ;;
        --out)         OUT_PATH="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--scenario FILE] [--num-uavs N] [--headless] [--out FILE]"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Create logs directory
mkdir -p "$(dirname "$OUT_PATH")"

echo "══════════════════════════════════════════════════════════════"
echo "  PUSHPAK UAV-X Swarm Simulation"
echo "  Scenario:  $SCENARIO"
echo "  UAVs:      $NUM_UAVS"
echo "  Headless:  $HEADLESS"
echo "  Log:       $OUT_PATH"
echo "══════════════════════════════════════════════════════════════"

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Source the swarm workspace (built with colcon in uav_swarm/ros2_ws)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
fi

# Launch
ros2 launch mission_bringup multi_uav_mission.launch.py \
    num_uavs:="$NUM_UAVS" \
    scenario:="$SCENARIO" \
    headless:="$HEADLESS"
