#!/bin/bash
# Start SITL with UDP output to Mission Planner in Parallels

echo "=========================================="
echo "SITL with Mission Planner (Parallels)"
echo "=========================================="
echo ""

# Use hardcoded Mission Planner VM IP
VM_IP="10.211.55.6"
echo "Using Mission Planner VM IP: $VM_IP"
echo ""

echo ""
echo "=========================================="
echo "Starting SITL with connections to:"
echo "  - Local (Mac):     127.0.0.1:14550"
echo "  - VM (Windows):    $VM_IP:14550"
echo "=========================================="
echo ""
echo "In Mission Planner:"
echo "  1. Click 'Connect' (top right)"
echo "  2. Select connection type: UDP"
echo "  3. Click 'Connect' (should auto-connect to port 14550)"
echo ""
echo "Then on your Mac, in another terminal run:"
echo "  cd $(pwd)"
echo "  python3 test_pixel_lock.py"
echo ""
echo "Starting SITL in 3 seconds..."
echo ""
sleep 3

# Start SITL with multiple outputs
# --out creates additional MAVLink outputs
./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --console \
    --map \
    --out=$VM_IP:14550

# Alternative if sim_vehicle.py doesn't work:
# ./build/sitl/bin/arducopter --model quad --home 42.3898,-71.1476,14,0 \
#     --serial0=tcp:0 \
#     --serial1=udp:127.0.0.1:14550 \
#     --serial2=udp:$VM_IP:14550
