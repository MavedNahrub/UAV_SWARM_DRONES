#!/usr/bin/env bash
###############################################################################
# step1_verify_ros2.sh — Verify ROS 2 Humble is correctly installed
###############################################################################
set -euo pipefail

echo "══════════════════════════════════════════════════════════════"
echo "  Step 1: Verifying ROS 2 Humble Installation"
echo "══════════════════════════════════════════════════════════════"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Check version
ROS_VERSION_OUT=$(ros2 --version 2>&1 || true)
echo "ros2 --version: $ROS_VERSION_OUT"

# Check distro
if [ "$ROS_DISTRO" != "humble" ]; then
    echo "❌ FAIL: ROS_DISTRO is '$ROS_DISTRO', expected 'humble'"
    exit 1
fi
echo "✅ ROS_DISTRO = humble"

# Check key packages
for pkg in rclpy rclcpp std_msgs geometry_msgs; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo "✅ Package '$pkg' found"
    else
        echo "❌ FAIL: Package '$pkg' not found"
        exit 1
    fi
done

# Check Python deps
python3 -c "import em; print(f'✅ empy version: {em.__version__}')" 2>/dev/null || \
    echo "⚠️  empy not importable (may be OK)"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ✅ Step 1 PASSED — ROS 2 Humble is correctly installed"
echo "══════════════════════════════════════════════════════════════"
