#!/usr/bin/env bash
###############################################################################
# step2_single_drone.sh — Launch single PX4 SITL drone with Gazebo Harmonic
#                          and verify it can take off
###############################################################################
set -euo pipefail

echo "══════════════════════════════════════════════════════════════"
echo "  Step 2: Single Drone SITL + Gazebo Harmonic"
echo "══════════════════════════════════════════════════════════════"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi

echo "📍 PX4 directory: $PX4_DIR"

# Check if SITL was already built
if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "🔧 Building PX4 SITL..."
    cd "$PX4_DIR"
    make px4_sitl
fi

echo ""
echo "🚀 Launching single drone simulation..."
echo "   Command: make px4_sitl gz_x500"
echo ""
echo "   Once Gazebo opens and the drone spawns:"
echo "   1. In the PX4 console, type: commander takeoff"
echo "   2. The drone should climb to ~2.5m"
echo "   3. Type: commander land"
echo "   4. Press Ctrl+C to exit"
echo ""
echo "══════════════════════════════════════════════════════════════"

cd "$PX4_DIR"
make px4_sitl gz_x500
