#!/usr/bin/env bash
set -e

echo "=============================================="
echo "  PX4 Preflight Verification Test"
echo "=============================================="

export PX4_HOME=/home/px4user/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH="${PX4_HOME}/Tools/simulation/gz/models:${PX4_HOME}/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_HOME}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_HOME}/src/modules/simulation/gz_bridge/server.config"

# Cleanup any stale processes
echo "[INFO] Cleaning up any stale processes..."
killall -9 px4 MicroXRCEAgent gz 2>/dev/null || true
sleep 1
rm -rf /tmp/px4* /tmp/px4_lock-* /tmp/px4-sock-* "${PX4_HOME}/build/px4_sitl_default/instance_0"

# 1. Start Gazebo headless
echo "[INFO] Starting Gazebo Harmonic with disaster_area.sdf (headless)..."
gz sim -s -r /home/px4user/uav_swarm/ros2_ws/src/mission_bringup/worlds/disaster_area.sdf &
GZ_PID=$!

# Wait for Gazebo
ATTEMPTS=30
while [ $ATTEMPTS -gt 0 ]; do
    if gz service -i --service /world/default/scene/info 2>&1 | grep -q "Service providers"; then
        echo "[INFO] Gazebo world is ready!"
        break
    fi
    ATTEMPTS=$((ATTEMPTS - 1))
    sleep 1
done

if [ $ATTEMPTS -eq 0 ]; then
    echo "[ERROR] Timed out waiting for Gazebo"
    kill $GZ_PID 2>/dev/null || true
    exit 1
fi

# 2. Start MicroXRCEAgent
echo "[INFO] Starting MicroXRCEAgent on port 8888..."
MicroXRCEAgent udp4 -p 8888 &
DDS_PID=$!

DDS_ATTEMPTS=20
while [ $DDS_ATTEMPTS -gt 0 ]; do
    if grep -q -i ":22B8 " /proc/net/udp 2>/dev/null; then
        echo "[INFO] MicroXRCEAgent listening on port 8888!"
        break
    fi
    DDS_ATTEMPTS=$((DDS_ATTEMPTS - 1))
    sleep 0.5
done

# 3. Start PX4 instance 0
echo "[INFO] Starting PX4 instance 0..."
mkdir -p "${PX4_HOME}/build/px4_sitl_default/instance_0"
cd "${PX4_HOME}/build/px4_sitl_default/instance_0"

PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_SIM_MODEL=gz_x500 \
PX4_GZ_WORLD=default \
PX4_GZ_MODELS="${PX4_HOME}/Tools/simulation/gz/models" \
PX4_GZ_WORLDS="${PX4_HOME}/Tools/simulation/gz/worlds" \
PX4_GZ_PLUGINS="${PX4_HOME}/build/px4_sitl_default/src/modules/simulation/gz_plugins" \
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}" \
GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}" \
GZ_SIM_SERVER_CONFIG_PATH="${GZ_SIM_SERVER_CONFIG_PATH}" \
"${PX4_HOME}/build/px4_sitl_default/bin/px4" -d -i 0 "${PX4_HOME}/ROMFS/px4fmu_common" > /tmp/px4_instance_0.log 2>&1 &
PX4_PID=$!

echo "[INFO] Waiting 20 seconds for PX4 boot and EKF2 / sensor convergence..."
sleep 20

echo "=== PX4 Log Output (last 40 lines) ==="
tail -n 40 /tmp/px4_instance_0.log

echo ""
echo "=== Running px4-commander --instance 0 check ==="
"${PX4_HOME}/build/px4_sitl_default/bin/px4-commander" --instance 0 check || true

echo ""
echo "=== Running px4-commander --instance 0 status ==="
"${PX4_HOME}/build/px4_sitl_default/bin/px4-commander" --instance 0 status || true

echo "[INFO] Cleaning up test processes..."
kill -9 $PX4_PID 2>/dev/null || true
kill -9 $DDS_PID 2>/dev/null || true
kill -9 $GZ_PID 2>/dev/null || true
killall -9 px4 MicroXRCEAgent gz 2>/dev/null || true
echo "=============================================="
echo "  Test Complete"
echo "=============================================="
