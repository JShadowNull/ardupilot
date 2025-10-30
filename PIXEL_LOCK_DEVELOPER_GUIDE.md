# PIXEL_LOCK Mode - Developer Integration Guide

**Version**: 1.1
**Date**: October 2024
**ArduPilot Version**: Copter 4.6.2
**ArduPilot Mode Number**: 29

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Companion Computer Requirements](#companion-computer-requirements)
4. [MAVLink Communication Protocol](#mavlink-communication-protocol)
5. [Vision Processing Requirements](#vision-processing-requirements)
6. [Message Format Details](#message-format-details)
7. [Control Loop Behavior](#control-loop-behavior)
8. [Configuration Parameters](#configuration-parameters)
9. [Testing & Validation](#testing--validation)
10. [Troubleshooting](#troubleshooting)
11. [Example Code](#example-code)

---

## Overview

PIXEL_LOCK is a custom ArduPilot flight mode (Mode 29) that enables vision-based target tracking and following without GPS dependency. The companion computer (e.g., Jetson) processes camera images, detects targets, and sends tracking commands to the flight controller via MAVLink.

**Key Features:**
- GPS-independent operation (uses visual feedback only)
- Real-time target tracking and following
- Configurable approach distance (via target size)
- Moving target support
- Velocity-based control with existing ArduPilot PIDs

**Use Cases:**
- Person following
- Vehicle tracking
- Object inspection
- Indoor GPS-denied flight

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    COMPANION COMPUTER                        │
│                      (Jetson/RPi)                            │
│                                                              │
│  ┌──────────────┐      ┌─────────────────────────────┐     │
│  │   Camera     │──────▶│  Vision Processing          │     │
│  │  (USB/CSI)   │      │  - Object detection         │     │
│  └──────────────┘      │  - Bounding box calculation │     │
│                        │  - Angular error conversion  │     │
│                        └─────────────┬───────────────┘     │
│                                      │                       │
│                        ┌─────────────▼───────────────┐     │
│                        │  MAVLink Message Builder     │     │
│                        │  - LANDING_TARGET (ID: 149)  │     │
│                        └─────────────┬───────────────┘     │
└────────────────────────────────────┼─────────────────────┘
                                      │ Serial/UDP (57600 baud)
                                      │ MAVLink 2.0
┌────────────────────────────────────▼─────────────────────┐
│                  FLIGHT CONTROLLER                        │
│                   (Matek H7A3-SLIM)                       │
│                                                            │
│  ┌──────────────────────────────────────────────────┐    │
│  │  PIXEL_LOCK Mode (mode.h / mode_pixel_lock.cpp)  │    │
│  │                                                    │    │
│  │  1. Receive LANDING_TARGET messages               │    │
│  │  2. Extract angle_x, angle_y, size_y              │    │
│  │  3. Apply deadband filtering                      │    │
│  │  4. Calculate velocity commands (NED frame)       │    │
│  │  5. Send to position controller                   │    │
│  │  6. Execute motor commands                        │    │
│  └────────────────────┬─────────────────────────────┘    │
│                       │                                    │
│  ┌────────────────────▼──────────────────┐               │
│  │   ArduPilot Controllers                │               │
│  │   - AC_PosControl (velocity)           │               │
│  │   - AC_AttitudeControl (thrust/yaw)    │               │
│  │   - AP_Motors (output)                 │               │
│  └────────────────────────────────────────┘               │
└────────────────────────────────────────────────────────────┘
```

---

## Companion Computer Requirements

### Hardware Requirements

**Minimum:**
- Jetson Nano / Raspberry Pi 4 (4GB+)
- Camera: 60° HFOV, 30+ FPS
- Serial connection to flight controller (TELEM1/TELEM2)

**Recommended:**
- Jetson Xavier NX / Orin Nano
- Camera: 60° HFOV, 60+ FPS, global shutter
- MAVLink over serial at 921600 baud (faster = better control)

### Software Requirements

- Python 3.7+ or C++ with MAVLink library
- Computer vision library (OpenCV, TensorRT, PyTorch)
- Object detection model (YOLO, SSD, custom)
- MAVLink library: `pymavlink` (Python) or `mavlink-c` (C++)

---

## MAVLink Communication Protocol

### Message Overview

PIXEL_LOCK mode receives target tracking data via the **LANDING_TARGET** MAVLink message (ID: 149).

**Message Frequency**: 20-50 Hz recommended (minimum 10 Hz)

**Message Flow**:
```
Companion Computer → Flight Controller
LANDING_TARGET (149) at 20-50 Hz
```

### Why LANDING_TARGET?

The LANDING_TARGET message is designed for precision landing but contains exactly the fields we need:
- `angle_x`: Horizontal angular error (radians)
- `angle_y`: Vertical angular error (radians)
- `size_y`: Target angular size (radians) - used for distance control
- `position_valid`: Target visibility flag

---

## Vision Processing Requirements

### 1. Target Detection

Your vision system must:
- Detect the target in the camera frame
- Calculate a bounding box around the target
- Track the target across frames (optional but recommended)

**Supported Detection Methods:**
- Object detection neural networks (YOLO, SSD, etc.)
- Traditional CV (color tracking, feature matching)
- Fiducial markers (ArUco, AprilTag)
- Person detection models

### 2. Coordinate System

**Camera Frame Convention**:
```
        Y (up)
        │
        │
        │
        └─────────── X (right)
       ╱
      ╱
     Z (forward, into scene)

Image Plane:
    0,0 ──────────── W
     │
     │    Center: (W/2, H/2)
     │
     │
     H
```

**Image coordinates**:
- Origin (0, 0) = Top-left corner
- X-axis = Right (0 to image_width)
- Y-axis = Down (0 to image_height)
- Center = (image_width/2, image_height/2)

### 3. Required Calculations

For each detected target, calculate:

#### A. Bounding Box Center (pixels)
```python
bbox_center_x = (bbox_x_min + bbox_x_max) / 2
bbox_center_y = (bbox_y_min + bbox_y_max) / 2
```

#### B. Pixel Error (pixels from image center)
```python
image_center_x = image_width / 2
image_center_y = image_height / 2

pixel_error_x = bbox_center_x - image_center_x  # Positive = target right
pixel_error_y = bbox_center_y - image_center_y  # Positive = target down
```

#### C. Angular Error (radians)
```python
import math

# Camera field of view (measure or look up in camera specs)
CAMERA_HFOV_DEG = 60.0  # Horizontal FOV in degrees
CAMERA_VFOV_DEG = 45.0  # Vertical FOV in degrees

# Convert to radians
hfov_rad = math.radians(CAMERA_HFOV_DEG)
vfov_rad = math.radians(CAMERA_VFOV_DEG)

# Normalize pixel error to [-1, 1]
normalized_error_x = pixel_error_x / (image_width / 2)
normalized_error_y = pixel_error_y / (image_height / 2)

# Convert to angular error (radians)
angle_x = normalized_error_x * (hfov_rad / 2)
angle_y = normalized_error_y * (vfov_rad / 2)
```

**Sign Convention**:
- `angle_x > 0`: Target is RIGHT of center → drone should move RIGHT
- `angle_x < 0`: Target is LEFT of center → drone should move LEFT
- `angle_y > 0`: Target is BELOW center → drone should DESCEND
- `angle_y < 0`: Target is ABOVE center → drone should CLIMB

#### D. Target Angular Size (radians)
```python
# Bounding box dimensions (pixels)
bbox_width = bbox_x_max - bbox_x_min
bbox_height = bbox_y_max - bbox_y_min

# Normalize to [0, 1]
normalized_height = bbox_height / image_height

# Convert to angular size (radians)
size_y = normalized_height * vfov_rad
```

**Purpose**: Used for distance control
- Small `size_y` = target far away → drone moves FORWARD
- Large `size_y` = target close → drone moves BACKWARD or stops
- Target size compared to `PLCK_TGT_SIZE` parameter (default 20°)

---

## Message Format Details

### LANDING_TARGET Message Structure

```python
from pymavlink import mavutil

# Create MAVLink connection
mav = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)

# Wait for heartbeat
mav.wait_heartbeat()

# Send LANDING_TARGET message
mav.mav.landing_target_send(
    time_usec,        # uint64_t: Timestamp (microseconds since UNIX epoch or system boot)
    target_num,       # uint8_t: Target ID (use 0)
    frame,            # uint8_t: Coordinate frame (use MAV_FRAME_BODY_FRD = 12)
    angle_x,          # float: Horizontal angular error (radians, + = right)
    angle_y,          # float: Vertical angular error (radians, + = down)
    distance,         # float: Distance to target (meters, can be 0 if unknown)
    size_x,           # float: Target size X (radians, can be 0)
    size_y,           # float: Target size Y (radians, IMPORTANT for distance control)
    position_valid,   # uint8_t: 0 = visible, 1+ = not visible
    type              # uint8_t: Type (use LANDING_TARGET_TYPE_VISION_OTHER = 3)
)
```

### Field-by-Field Specification

| Field | Type | Value | Description |
|-------|------|-------|-------------|
| `time_usec` | uint64 | `time.time() * 1e6` | Timestamp in microseconds. Use system time or monotonic time. |
| `target_num` | uint8 | `0` | Target ID. Always use 0 for single target. |
| `frame` | uint8 | `12` | MAV_FRAME_BODY_FRD (12). Always use body frame. |
| `angle_x` | float | `-π/2 to +π/2` | **Horizontal angular error** in radians. Positive = target right of center. |
| `angle_y` | float | `-π/2 to +π/2` | **Vertical angular error** in radians. Positive = target below center. |
| `distance` | float | `> 0` or `0` | Distance to target in meters. Set to 0 if unknown (not currently used by PIXEL_LOCK). |
| `size_x` | float | `> 0` or `0` | Target width in radians. Can be 0 (not currently used). |
| `size_y` | float | `> 0` | **Target height in radians**. CRITICAL for distance control! |
| `position_valid` | uint8 | `0` or `1` | **0 = target visible**, 1+ = target lost. ArduPilot checks `visible = (position_valid == 0)`. |
| `type` | uint8 | `3` | LANDING_TARGET_TYPE_VISION_OTHER (3). Always use vision type. |

### Critical Fields Summary

**MUST provide accurately:**
1. ✅ `angle_x` - Controls left/right movement
2. ✅ `angle_y` - Controls up/down movement
3. ✅ `size_y` - Controls forward/backward (distance to target)
4. ✅ `position_valid` - Controls hover vs. track behavior

**Can approximate or set to 0:**
- `distance` - Not used by control algorithm
- `size_x` - Not used by control algorithm

---

## Control Loop Behavior

### How PIXEL_LOCK Uses Your Data

```python
# Pseudocode of what happens in mode_pixel_lock.cpp

# 1. Check if vision data is fresh and valid
if (time_since_update < PLCK_TIMEOUT) and (position_valid == 0):
    vision_valid = True
else:
    vision_valid = False

# 2. If vision valid, calculate velocities
if vision_valid:
    # Apply deadband (ignore small errors)
    angle_x = apply_deadband(angle_x, PLCK_DEADBAND)
    angle_y = apply_deadband(angle_y, PLCK_DEADBAND)

    # Horizontal velocity (left/right in body frame)
    vel_right = angle_x * PLCK_VEL_GAIN
    vel_right = constrain(vel_right, -PLCK_MAX_VEL, +PLCK_MAX_VEL)

    # Forward/back velocity based on SIZE error
    size_error = size_y - PLCK_TGT_SIZE  # Compare to desired size
    size_error = apply_deadband(size_error, PLCK_SIZE_DB)
    vel_forward = -size_error * PLCK_VEL_GAIN * 2.0
    vel_forward = constrain(vel_forward, -PLCK_MAX_VEL, +PLCK_MAX_VEL)

    # Vertical velocity (climb/descend)
    climb_rate = -angle_y * PLCK_VEL_GAIN
    climb_rate = constrain(climb_rate, -PLCK_MAX_CLIMB, +PLCK_MAX_CLIMB)

    # Yaw rate (turn to face target)
    yaw_rate = angle_x * 0.5

    # Rotate body velocities to world (NED) frame
    vel_north = vel_forward * cos(yaw) - vel_right * sin(yaw)
    vel_east = vel_forward * sin(yaw) + vel_right * cos(yaw)

else:
    # Vision lost - hover in place (or drift if no GPS)
    vel_north = 0
    vel_east = 0
    climb_rate = 0
    yaw_rate = 0

# 3. Send to position controller
position_controller.set_velocity_NE(vel_north, vel_east)
position_controller.set_velocity_D(climb_rate)
attitude_controller.set_yaw_rate(yaw_rate)
```

### Control Algorithm Summary

**Horizontal Control** (Left/Right):
- Input: `angle_x` (radians)
- Output: Velocity proportional to error
- Small error = slow movement, large error = fast movement
- Max velocity limited by `PLCK_MAX_VEL` (default 3 m/s)

**Vertical Control** (Up/Down):
- Input: `angle_y` (radians)
- Output: Climb rate proportional to error
- Max climb rate limited by `PLCK_MAX_CLIMB` (default 1.5 m/s)

**Distance Control** (Forward/Back):
- Input: `size_y` (radians) vs. `PLCK_TGT_SIZE` (default 20°)
- If bbox too small → move forward
- If bbox too large → move backward
- Maintains target at configured standoff distance

**Yaw Control**:
- Slowly rotates to face target (0.5 rad/s gain on angle_x)
- Helps keep target centered and improves camera tracking

---

## Configuration Parameters

All parameters accessible in Mission Planner under `PLCK_*`:

| Parameter | Default | Units | Range | Description |
|-----------|---------|-------|-------|-------------|
| `PLCK_VEL_GAIN` | 2.0 | - | 0.5-5.0 | Velocity gain. Higher = more aggressive movement. |
| `PLCK_MAX_VEL` | 3.0 | m/s | 1.0-5.0 | Maximum horizontal velocity. |
| `PLCK_MAX_CLIMB` | 1.5 | m/s | 0.5-3.0 | Maximum climb/descent rate. |
| `PLCK_DEADBAND` | 2.0 | deg | 0.5-5.0 | Centering deadband (ignore small errors). |
| `PLCK_TGT_SIZE` | 20.0 | deg | 5.0-45.0 | **Desired target angular size** (controls standoff distance). |
| `PLCK_SIZE_DB` | 2.0 | deg | 0.5-5.0 | Size control deadband. |
| `PLCK_TIMEOUT` | 500 | ms | 100-2000 | Vision data timeout (how long before considering vision lost). |

### Tuning Guidance

**For Slower, Smoother Tracking:**
- Decrease `PLCK_VEL_GAIN` → 1.0
- Decrease `PLCK_MAX_VEL` → 1.5 m/s
- Increase `PLCK_DEADBAND` → 3.0°

**For Faster, More Aggressive Tracking:**
- Increase `PLCK_VEL_GAIN` → 3.0
- Increase `PLCK_MAX_VEL` → 5.0 m/s
- Decrease `PLCK_DEADBAND` → 1.0°

**For Closer Approach:**
- Increase `PLCK_TGT_SIZE` → 30° (target fills more of screen)

**For Farther Standoff:**
- Decrease `PLCK_TGT_SIZE` → 10° (target smaller in screen)

---

## Testing & Validation

### Test Environment Setup

1. **SITL Testing** (Software-in-the-Loop):
   - Use provided `test_pixel_lock.py` script
   - Simulates camera and sends LANDING_TARGET messages
   - Safe testing without hardware risk

2. **Bench Testing**:
   - Props off, drone on bench
   - Run companion computer vision
   - Move target around, verify messages sent
   - Check Mission Planner parameter screen for PLCK values

3. **Flight Testing**:
   - Start with high altitude (10-15m)
   - Use large, easy-to-detect target
   - Have manual pilot ready to take over
   - Start with low `PLCK_VEL_GAIN` (1.0) for gentle movement

### Validation Checklist

**Vision System:**
- [ ] Target detected reliably (>90% detection rate)
- [ ] Bounding box stable (not jittering excessively)
- [ ] Tracking persists during occlusion/motion blur
- [ ] Angular calculations verified against known target positions

**MAVLink Communication:**
- [ ] Messages sent at ≥20 Hz
- [ ] Flight controller receives messages (check MAVLink Inspector)
- [ ] `position_valid` = 0 when target visible
- [ ] `angle_x`, `angle_y`, `size_y` reasonable values (-1 to +1 rad typical)

**Flight Behavior:**
- [ ] Drone moves toward target when far away
- [ ] Drone stops when target reaches desired size
- [ ] Drone centers target horizontally and vertically
- [ ] Drone hovers when target lost (GPS) or drifts (no GPS)

### Debug Messages

Enable MAVLink message logging in Mission Planner:
```
CONFIG → Planner → Enable telemetry logging
```

Monitor incoming `LANDING_TARGET` messages in real-time:
```
CTRL+F → MAVLink Inspector → Filter: LANDING_TARGET
```

---

## Troubleshooting

### Problem: Drone doesn't move in PIXEL_LOCK mode

**Possible Causes:**
1. ❌ No LANDING_TARGET messages being received
   - **Check**: MAVLink Inspector for LANDING_TARGET messages
   - **Fix**: Verify serial connection, baud rate, MAVLink routing

2. ❌ `position_valid` != 0 (target marked as not visible)
   - **Check**: Set `position_valid = 0` in your messages
   - **Fix**: Only set `position_valid = 1` when target truly lost

3. ❌ Vision timeout (messages too slow or stopped)
   - **Check**: Message rate < 2 Hz or stopped
   - **Fix**: Increase message rate to ≥20 Hz

4. ❌ Angular errors too small (within deadband)
   - **Check**: `angle_x`, `angle_y` < `PLCK_DEADBAND` (2° default)
   - **Fix**: Move target farther from center or decrease `PLCK_DEADBAND`

### Problem: Drone moves but in wrong direction

**Possible Causes:**
1. ❌ Angular error sign inverted
   - **Check**: Target right of center should give `angle_x > 0`
   - **Fix**: Verify pixel error calculation (target_x - center_x)

2. ❌ Camera mounted backwards/upside-down
   - **Check**: Physical camera orientation
   - **Fix**: Adjust angular error calculations for camera mounting

### Problem: Drone oscillates or unstable

**Possible Causes:**
1. ❌ `PLCK_VEL_GAIN` too high
   - **Fix**: Reduce to 1.0 and increase slowly

2. ❌ Bounding box jittering
   - **Fix**: Add temporal filtering/smoothing to bbox

3. ❌ Message rate too low
   - **Fix**: Increase to 30-50 Hz

### Problem: Drone too slow to reach target

**Possible Causes:**
1. ❌ `PLCK_VEL_GAIN` too low
   - **Fix**: Increase to 2.5-3.0

2. ❌ `PLCK_MAX_VEL` too conservative
   - **Fix**: Increase to 4-5 m/s

3. ❌ Distance control not working (size_y incorrect)
   - **Check**: `size_y` should increase as drone approaches
   - **Fix**: Verify bbox height → angular size calculation

---

## Example Code

### Complete Python Example (Jetson/RPi)

```python
#!/usr/bin/env python3
"""
PIXEL_LOCK Companion Computer Example
Detects person, calculates angular errors, sends LANDING_TARGET messages
"""

import cv2
import numpy as np
import time
import math
from pymavlink import mavutil

# ==================== CONFIGURATION ====================

# Camera parameters (measure or look up in camera datasheet)
CAMERA_HFOV_DEG = 60.0  # Horizontal field of view (degrees)
CAMERA_VFOV_DEG = 45.0  # Vertical field of view (degrees)
IMAGE_WIDTH = 640       # Camera resolution width
IMAGE_HEIGHT = 480      # Camera resolution height

# MAVLink connection
MAVLINK_PORT = '/dev/ttyTHS1'  # Jetson UART (use '/dev/ttyUSB0' for USB)
MAVLINK_BAUD = 921600

# Detection parameters
TARGET_CLASS = 0  # YOLO class ID (0 = person for COCO dataset)

# ==================== INITIALIZATION ====================

# Connect to flight controller
print("Connecting to flight controller...")
mav = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)
mav.wait_heartbeat()
print(f"✓ Connected to system {mav.target_system}")

# Open camera
print("Opening camera...")
cap = cv2.VideoCapture(0)  # Use 0 for USB camera, adjust for CSI
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("✗ Failed to open camera!")
    exit(1)

print("✓ Camera opened")

# Load object detection model (example: YOLO)
# Replace with your actual model loading code
print("Loading detection model...")
# net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
# model = load_your_model()
print("✓ Model loaded (placeholder - implement your model)")

# ==================== UTILITY FUNCTIONS ====================

def calculate_angular_errors(bbox_center_x, bbox_center_y, bbox_height):
    """
    Convert bounding box to angular errors for LANDING_TARGET message.

    Args:
        bbox_center_x: Bounding box center X coordinate (pixels)
        bbox_center_y: Bounding box center Y coordinate (pixels)
        bbox_height: Bounding box height (pixels)

    Returns:
        tuple: (angle_x, angle_y, size_y) in radians
    """
    # Image center
    center_x = IMAGE_WIDTH / 2.0
    center_y = IMAGE_HEIGHT / 2.0

    # Pixel error (+ right, + down)
    pixel_error_x = bbox_center_x - center_x
    pixel_error_y = bbox_center_y - center_y

    # Normalize to [-1, +1]
    normalized_x = pixel_error_x / (IMAGE_WIDTH / 2.0)
    normalized_y = pixel_error_y / (IMAGE_HEIGHT / 2.0)

    # Convert to radians
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    vfov_rad = math.radians(CAMERA_VFOV_DEG)

    angle_x = normalized_x * (hfov_rad / 2.0)
    angle_y = normalized_y * (vfov_rad / 2.0)

    # Calculate angular size
    normalized_height = bbox_height / IMAGE_HEIGHT
    size_y = normalized_height * vfov_rad

    return angle_x, angle_y, size_y


def detect_target(frame):
    """
    Detect target in frame and return bounding box.

    Args:
        frame: OpenCV image (numpy array)

    Returns:
        tuple: (detected, x_center, y_center, height) or (False, 0, 0, 0)
    """
    # ============================================================
    # IMPLEMENT YOUR DETECTION MODEL HERE
    # ============================================================
    # This is a placeholder - replace with your actual detection code
    # Examples:
    #   - YOLO/SSD neural network
    #   - Traditional CV (color detection, template matching)
    #   - ArUco marker detection
    #   - Person detection model

    # Example: Simple color-based detection (BGR: blue object)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return False, 0, 0, 0

    # Get largest contour
    largest_contour = max(contours, key=cv2.contourArea)

    # Get bounding box
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Calculate center
    center_x = x + w / 2.0
    center_y = y + h / 2.0

    # Minimum size filter (avoid noise)
    if w < 20 or h < 20:
        return False, 0, 0, 0

    return True, center_x, center_y, h


def send_landing_target(angle_x, angle_y, size_y, visible):
    """
    Send LANDING_TARGET MAVLink message to flight controller.

    Args:
        angle_x: Horizontal angular error (radians, + right)
        angle_y: Vertical angular error (radians, + down)
        size_y: Target angular size (radians)
        visible: True if target visible, False if lost
    """
    timestamp_us = int(time.time() * 1e6)

    mav.mav.landing_target_send(
        timestamp_us,                                    # time_usec
        0,                                               # target_num (always 0)
        mavutil.mavlink.MAV_FRAME_BODY_FRD,             # frame (12 = body frame)
        float(angle_x),                                  # angle_x (radians)
        float(angle_y),                                  # angle_y (radians)
        0.0,                                            # distance (not used, set to 0)
        0.0,                                            # size_x (not used, set to 0)
        float(size_y),                                   # size_y (IMPORTANT!)
        0 if visible else 1,                            # position_valid (0=visible)
        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_OTHER # type (3)
    )


# ==================== MAIN LOOP ====================

print("\n" + "="*60)
print("PIXEL_LOCK Vision System Running")
print("="*60 + "\n")

frame_count = 0
last_fps_time = time.time()
fps = 0

try:
    while True:
        # Read frame from camera
        ret, frame = cap.read()
        if not ret:
            print("✗ Failed to read frame")
            continue

        # Detect target
        detected, center_x, center_y, height = detect_target(frame)

        if detected:
            # Calculate angular errors
            angle_x, angle_y, size_y = calculate_angular_errors(
                center_x, center_y, height
            )

            # Send MAVLink message
            send_landing_target(angle_x, angle_y, size_y, visible=True)

            # Draw bounding box on frame (for debugging)
            x = int(center_x - 50)  # Approximate box from center and height
            y = int(center_y - height/2)
            w = 100  # Approximate width
            h = int(height)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

            # Display info
            info_text = f"X:{angle_x*57.3:.1f}° Y:{angle_y*57.3:.1f}° Size:{size_y*57.3:.1f}°"
            cv2.putText(frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Target lost - send message with position_valid=1
            send_landing_target(0.0, 0.0, 0.0, visible=False)

            # Display "LOST" on frame
            cv2.putText(frame, "TARGET LOST", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Calculate FPS
        frame_count += 1
        if frame_count % 10 == 0:
            current_time = time.time()
            fps = 10 / (current_time - last_fps_time)
            last_fps_time = current_time

        # Display FPS
        cv2.putText(frame, f"FPS: {fps:.1f}", (IMAGE_WIDTH-120, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Draw center crosshair
        cv2.line(frame, (IMAGE_WIDTH//2 - 20, IMAGE_HEIGHT//2),
                (IMAGE_WIDTH//2 + 20, IMAGE_HEIGHT//2), (255, 255, 255), 1)
        cv2.line(frame, (IMAGE_WIDTH//2, IMAGE_HEIGHT//2 - 20),
                (IMAGE_WIDTH//2, IMAGE_HEIGHT//2 + 20), (255, 255, 255), 1)

        # Show frame (comment out for headless operation)
        cv2.imshow('PIXEL_LOCK Vision', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n\nStopping vision system...")

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("✓ Vision system stopped")
```

### C++ Example (Snippet)

```cpp
#include <iostream>
#include <chrono>
#include <cmath>
#include <mavlink/v2.0/common/mavlink.h>

// Camera parameters
const float CAMERA_HFOV_DEG = 60.0f;
const float CAMERA_VFOV_DEG = 45.0f;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;

struct AngularErrors {
    float angle_x;
    float angle_y;
    float size_y;
};

AngularErrors calculate_angular_errors(float bbox_center_x, float bbox_center_y, float bbox_height) {
    // Image center
    float center_x = IMAGE_WIDTH / 2.0f;
    float center_y = IMAGE_HEIGHT / 2.0f;

    // Pixel error
    float pixel_error_x = bbox_center_x - center_x;
    float pixel_error_y = bbox_center_y - center_y;

    // Normalize
    float normalized_x = pixel_error_x / (IMAGE_WIDTH / 2.0f);
    float normalized_y = pixel_error_y / (IMAGE_HEIGHT / 2.0f);

    // Convert to radians
    float hfov_rad = CAMERA_HFOV_DEG * M_PI / 180.0f;
    float vfov_rad = CAMERA_VFOV_DEG * M_PI / 180.0f;

    AngularErrors errors;
    errors.angle_x = normalized_x * (hfov_rad / 2.0f);
    errors.angle_y = normalized_y * (vfov_rad / 2.0f);

    // Angular size
    float normalized_height = bbox_height / IMAGE_HEIGHT;
    errors.size_y = normalized_height * vfov_rad;

    return errors;
}

void send_landing_target(int serial_fd, float angle_x, float angle_y, float size_y, bool visible) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Get timestamp
    auto now = std::chrono::system_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    // Pack message
    mavlink_msg_landing_target_pack(
        255,                                      // System ID (companion computer)
        MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,   // Component ID
        &msg,
        us,                                       // time_usec
        0,                                        // target_num
        MAV_FRAME_BODY_FRD,                      // frame
        angle_x,                                  // angle_x
        angle_y,                                  // angle_y
        0.0f,                                    // distance
        0.0f,                                    // size_x
        size_y,                                   // size_y
        visible ? 0 : 1,                         // position_valid (0=visible)
        LANDING_TARGET_TYPE_VISION_OTHER         // type
    );

    // Serialize and send
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(serial_fd, buf, len);
}
```

---

## FAQ

**Q: What happens if I don't send `size_y`?**
A: The drone won't know how far away the target is. It will only center the target horizontally/vertically but won't maintain standoff distance. It may fly too close or stay too far.

**Q: Can I use this indoors without GPS?**
A: Yes! PIXEL_LOCK doesn't require GPS for tracking. However, when vision is lost, the drone needs a position reference (GPS, optical flow, or VIO) to hold position. Without any reference, it will drift.

**Q: What frame rate should I send messages?**
A: Recommended 20-50 Hz. Minimum 10 Hz. The `PLCK_TIMEOUT` parameter (default 500ms) determines how long before vision is considered lost.

**Q: How do I handle multiple targets?**
A: PIXEL_LOCK currently supports single target. To handle multiple targets, your vision system should:
- Select one target (closest, specific person, etc.)
- Send LANDING_TARGET for that target only
- Switch targets smoothly with hysteresis to avoid oscillation

**Q: Can the target move?**
A: Yes! PIXEL_LOCK is designed to track moving targets. The test script includes moving target simulation (walking person, circular motion, etc.).

**Q: What if my camera has a different FOV?**
A: Measure or look up your camera's FOV specifications and update `CAMERA_HFOV_DEG` and `CAMERA_VFOV_DEG` in your code. Common values:
- Pi Camera v2: 62.2° H × 48.8° V
- Logitech C920: 78° H × 53° V
- Wide-angle action cams: 120°+ H

---

## Additional Resources

**ArduPilot Documentation:**
- https://ardupilot.org/dev/docs/building-the-code.html
- https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages.html

**MAVLink Protocol:**
- https://mavlink.io/en/messages/common.html#LANDING_TARGET
- https://mavlink.io/en/services/landing_target.html

**Computer Vision:**
- YOLO object detection: https://github.com/ultralytics/yolov5
- OpenCV tutorials: https://docs.opencv.org/4.x/d9/df8/tutorial_root.html

**Hardware Setup:**
- Jetson Nano setup: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
- MAVLink serial connection: https://ardupilot.org/dev/docs/companion-computers.html

---

## Contact & Support

For questions about PIXEL_LOCK mode implementation:
- Check ArduPilot forums: https://discuss.ardupilot.org/
- Review source code: `ArduCopter/mode_pixel_lock.cpp`

---

**Document Version**: 1.1
**Last Updated**: October 2024
**Tested ArduPilot Version**: Copter 4.6.2
**Tested Hardware**: Matek H7A3-SLIM, Jetson Nano, Pi Camera v2

**Implementation Notes**:
- Uses ArduPilot 4.6.2 position controller API (`input_vel_accel_xy`, `input_vel_accel_z`)
- Velocity commands in cm/s (converted internally from m/s)
- Attitude control uses degrees for yaw rate
- Compatible with existing ArduPilot PIDs and safety features
