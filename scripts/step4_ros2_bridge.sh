#!/usr/bin/env bash
###############################################################################
# step4_ros2_bridge.sh — Verify ROS 2 bridge with namespaced topics
#
# Launches:
#   1. Micro XRCE-DDS Agent on UDP port 8888
#   2. Two PX4 SITL instances
#   3. Checks ros2 topic list for namespaced topics
###############################################################################
set -euo pipefail

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

echo "══════════════════════════════════════════════════════════════"
echo "  Step 4: ROS 2 Bridge with Namespaced Topics"
echo "══════════════════════════════════════════════════════════════"

SESSION="px4_ros2"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 200 -y 50

# ── Pane 0: XRCE-DDS Agent ──
tmux send-keys -t "$SESSION" "MicroXRCEAgent udp4 -p 8888" Enter
sleep 2

# ── Pane 1: Drone 0 ──
tmux split-window -v -t "$SESSION"
tmux send-keys -t "$SESSION" \
    "cd $PX4_DIR && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='0,0' $PX4_BIN -i 0" Enter

echo "⏳ Waiting 20s for Gazebo + Drone 0..."
sleep 20

# ── Pane 2: Drone 1 ──
tmux split-window -v -t "$SESSION"
tmux send-keys -t "$SESSION" \
    "cd $PX4_DIR && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='3,0' $PX4_BIN -i 1" Enter

echo "⏳ Waiting 15s for Drone 1..."
sleep 15

# ── Verify topics ──
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Checking ROS 2 topics..."
echo "══════════════════════════════════════════════════════════════"

TOPIC_LIST=$(ros2 topic list 2>/dev/null || echo "")
echo "$TOPIC_LIST"
echo ""

# Check for expected namespaced topics
PASS=true

# Instance 0 topics (no namespace prefix)
if echo "$TOPIC_LIST" | grep -q "/fmu/out/vehicle_status"; then
    echo "✅ Drone 0: /fmu/out/vehicle_status found"
else
    echo "❌ Drone 0: /fmu/out/vehicle_status NOT found"
    PASS=false
fi

# Instance 1 topics (px4_1 namespace)
if echo "$TOPIC_LIST" | grep -q "/px4_1/fmu/out/vehicle_status"; then
    echo "✅ Drone 1: /px4_1/fmu/out/vehicle_status found"
else
    echo "❌ Drone 1: /px4_1/fmu/out/vehicle_status NOT found"
    PASS=false
fi

echo ""
if [ "$PASS" = true ]; then
    echo "══════════════════════════════════════════════════════════════"
    echo "  ✅ Step 4 PASSED — Both drones have namespaced ROS 2 topics"
    echo "══════════════════════════════════════════════════════════════"
else
    echo "══════════════════════════════════════════════════════════════"
    echo "  ❌ Step 4 FAILED — Some topics are missing"
    echo "  Check the tmux session: tmux attach -t $SESSION"
    echo "══════════════════════════════════════════════════════════════"
fi

echo ""
echo "  tmux session: $SESSION"
echo "    Attach: tmux attach -t $SESSION"
echo "    Kill:   tmux kill-session -t $SESSION"
