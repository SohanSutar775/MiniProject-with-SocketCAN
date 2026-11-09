#!/bin/bash
# Create the virtual CAN interface (needs root). Usage: sudo ./setup_vcan.sh [vcan0]
IF=${1:-vcan0}
modprobe vcan
ip link add dev $IF type vcan 2>/dev/null || true
ip link set up $IF
ip -details link show $IF
