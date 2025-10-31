#!/bin/bash
# Start SITL with UDP output to Mission Planner in Parallels

echo "=========================================="
echo "SITL with Mission Planner (Parallels)"
echo "=========================================="
echo ""

# Use hardcoded Mission Planner VM IP
VM_IP="10.211.55.6"
echo ""
echo "=========================================="
echo "Starting SITL with connections to:"
echo "  - Local (Mac):     127.0.0.1:14550"
echo "  - VM (Windows):    $VM_IP:14550"
echo "=========================================="
echo ""

# Start SITL with multiple outputs
# --out creates additional MAVLink outputs
./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --out=$VM_IP:14550
