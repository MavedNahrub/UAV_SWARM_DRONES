#!/usr/bin/env bash
###############################################################################
# step3_two_drones.sh — Launch 2 PX4 SITL instances in one Gazebo world
#
# Architecture:
#   Instance 0: MAVLink UDP ports 14540/14580 (default)
#   Instance 1: MAVLink UDP ports 14541/14581 (auto-offset)
#
# Both share one Gazebo Harmonic server process.
###############################################################################
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

echo "══════════════════════════════════════════════════════════════"
echo "  Step 3: Two PX4 SITL Instances in One Gazebo World"
echo "══════════════════════════════════════════════════════════════"

if [ ! -f "$PX4_BIN" ]; then
    echo "🔧 Building PX4 SITL first..."
    cd "$PX4_DIR" && make px4_sitl
fi

# Use tmux for multi-pane management
SESSION="px4_multi"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 200 -y 50

# ── Pane 0: Drone 0 (spawns Gazebo automatically) ──
tmux send-keys -t "$SESSION" \
    "mkdir -p $PX4_DIR/build/px4_sitl_default/instance_0 && cd $PX4_DIR/build/px4_sitl_default/instance_0 && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='0,0' $PX4_BIN -i 0" Enter

echo "⏳ Waiting 15s for Gazebo + Drone 0 to start..."
sleep 15

# ── Pane 1: Drone 1 (joins existing Gazebo world) ──
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" \
    "mkdir -p $PX4_DIR/build/px4_sitl_default/instance_1 && cd $PX4_DIR/build/px4_sitl_default/instance_1 && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='2,0' $PX4_BIN -i 1" Enter

echo "⏳ Waiting 10s for Drone 1 to spawn..."
sleep 10

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ✅ Two drones should now be visible in Gazebo"
echo ""
echo "  To verify independent control:"
echo "    In tmux pane 0 (left):  commander takeoff"
echo "    In tmux pane 1 (right): (wait, don't takeoff yet)"
echo "    → Only drone 0 should lift off"
echo ""
echo "  tmux session: $SESSION"
echo "    Switch panes: Ctrl+B then arrow keys"
echo "    Kill session:  tmux kill-session -t $SESSION"
echo "══════════════════════════════════════════════════════════════"

tmux attach -t "$SESSION"
