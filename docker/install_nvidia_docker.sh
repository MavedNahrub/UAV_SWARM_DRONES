#!/usr/bin/env bash
###############################################################################
# install_nvidia_docker.sh — Install NVIDIA Container Toolkit for GPU access
#
# Run this on the HOST (Ubuntu 26.04), NOT inside Docker.
# Requires: NVIDIA driver already installed (nvidia-smi must work)
###############################################################################
set -euo pipefail

echo "══════════════════════════════════════════════════════════════"
echo "  Installing NVIDIA Container Toolkit"
echo "══════════════════════════════════════════════════════════════"

# Check NVIDIA driver
if ! nvidia-smi &>/dev/null; then
    echo "❌ nvidia-smi failed — install NVIDIA drivers first"
    exit 1
fi
echo "✅ NVIDIA driver detected"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

# Add NVIDIA container toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ✅ NVIDIA Container Toolkit installed"
echo "  Test: docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi"
echo "══════════════════════════════════════════════════════════════"
