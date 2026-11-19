#!/bin/bash
# Creates the virtual CAN bus vcan0 (run after each reboot).
set -e
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
echo "vcan0 is up"
