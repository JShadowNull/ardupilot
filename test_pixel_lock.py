#!/usr/bin/env python3
"""
PIXEL_LOCK MOVING TARGET TEST - Ground Person Tracking
- ArduPilot Version: Copter 4.6.2
- Mode Number: 29 (PIXEL_LOCK)
- Drone takes off to 15m altitude
- Person starts 50m ahead on the ground (5'10" / 1.78m tall)
- Person walks at 1.5 m/s (~3.4 mph) in a 40m path back and forth
- Drone tracks and follows from 15m altitude
- Tests long-distance moving target tracking from altitude
- Configurable motion patterns (line/circle/figure8) and speeds

Test validates:
- LANDING_TARGET message reception
- Angular error to velocity conversion
- Distance control via target size
- Moving target tracking capability
"""

import time
import math
import numpy as np
from pymavlink import mavutil
import sys
from datetime import datetime

# Setup logging to both console and file
LOG_FILE = f"pixel_lock_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

class Logger:
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, 'w', buffering=1)  # Line buffered

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

sys.stdout = Logger(LOG_FILE)
sys.stderr = sys.stdout

print(f"[{datetime.now().strftime('%H:%M:%S')}] Logging to: {LOG_FILE}")
print("=" * 80)

# Connection parameters
CONNECTION_STRING = 'udp:127.0.0.1:14550'

# Camera parameters
CAMERA_HFOV_DEG = 60.0
CAMERA_VFOV_DEG = 45.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_PITCH_ANGLE_DEG = 25.0  # FPV camera tilted upward (positive = up)

# Test parameters
TAKEOFF_ALT = 15.0  # meters - high altitude to track ground target
TEST_DURATION = 60.0  # seconds - 1 minute to track person walking
LOCK_THRESHOLD_PIXELS_X = 10.0
LOCK_THRESHOLD_PIXELS_Y = 10.0
LOCK_TIME_REQUIRED = 2.0  # seconds

# Moving target parameters - person walking on ground
TARGET_MOTION_TYPE = "line"    # Options: "circle", "figure8", "line"
TARGET_START_OFFSET_N = 100.0   # Starting position: 50m North of drone
TARGET_START_OFFSET_E = -25.0  # Starting position: 25m West (will walk 50m East)
TARGET_ALTITUDE_OFFSET = 15.0  # Target on GROUND (drone at 15m, so offset = 15m DOWN)
TARGET_SPEED = 5             # m/s - walking speed (~3.4 mph / 5.4 kph)
TARGET_CIRCLE_RADIUS = 200   # meters - walking distance (50m round trip)
TARGET_HEIGHT = 1.78           # meters - target height: 1.78m (person height)

class DroneState:
    """Track actual drone state from telemetry"""
    def __init__(self):
        self.armed = False
        self.mode = None

        # Position (NED frame, meters from home)
        self.pos_n = 0.0
        self.pos_e = 0.0
        self.pos_d = 0.0

        # Attitude (radians)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Velocity (NED frame, m/s)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.last_update = 0

        # Track starting position for distance/bearing calculations
        self.start_n = 0.0
        self.start_e = 0.0
        self.start_yaw = 0.0
        self.start_initialized = False

    def update_from_heartbeat(self, msg):
        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.mode = msg.custom_mode

    def update_from_attitude(self, msg):
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
        self.last_update = time.time()

    def update_from_vfr_hud(self, msg):
        """Update heading from VFR_HUD (more reliable, in degrees)"""
        # VFR_HUD.heading is in degrees (0-360)
        self.yaw = math.radians(msg.heading)  # Convert to radians for consistency
        self.last_update = time.time()

    def update_from_local_position(self, msg):
        """Update from LOCAL_POSITION_NED message"""
        self.pos_n = msg.x
        self.pos_e = msg.y
        self.pos_d = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.last_update = time.time()

    @property
    def altitude(self):
        return -self.pos_d  # NED: negative down = positive altitude

    def get_heading_deg(self):
        """Get heading in degrees (0-360, 0=North, 90=East)"""
        heading_deg = math.degrees(self.yaw)
        # Normalize to 0-360
        heading_deg = heading_deg % 360
        if heading_deg < 0:
            heading_deg += 360
        return heading_deg

    def get_travel_distance(self):
        """Get total distance traveled from start position"""
        if not self.start_initialized:
            return 0.0
        dn = self.pos_n - self.start_n
        de = self.pos_e - self.start_e
        return math.sqrt(dn*dn + de*de)

    def get_travel_bearing(self):
        """Get bearing of travel from start to current position (0-360°, 0=North)"""
        if not self.start_initialized:
            return 0.0
        dn = self.pos_n - self.start_n
        de = self.pos_e - self.start_e
        if abs(dn) < 0.01 and abs(de) < 0.01:
            return 0.0  # Haven't moved
        bearing_rad = math.atan2(de, dn)  # atan2(E, N) gives bearing from North
        bearing_deg = math.degrees(bearing_rad)
        # Normalize to 0-360
        bearing_deg = bearing_deg % 360
        if bearing_deg < 0:
            bearing_deg += 360
        return bearing_deg

    def initialize_start_position(self):
        """Call this when PIXEL_LOCK mode starts to set the reference point"""
        self.start_n = self.pos_n
        self.start_e = self.pos_e
        self.start_yaw = self.yaw
        self.start_initialized = True

def rotation_matrix_body_to_ned(roll, pitch, yaw):
    """
    Create rotation matrix from body frame to NED frame
    Body frame: X-forward, Y-right, Z-down
    NED frame: X-north, Y-east, Z-down
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Rotation matrix from body to NED
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])
    return R

def world_point_to_camera_frame(target_ned, drone_state):
    """
    Calculate where a world point appears in camera frame

    Args:
        target_ned: [N, E, D] target position in NED frame (meters)
        drone_state: DroneState with position and attitude

    Returns:
        (angle_x_rad, angle_y_rad, distance_m) or None if behind camera
    """
    # Vector from drone to target in NED frame
    target_vec_ned = np.array([
        target_ned[0] - drone_state.pos_n,
        target_ned[1] - drone_state.pos_e,
        target_ned[2] - drone_state.pos_d
    ])

    # Rotate to body frame
    R_body_to_ned = rotation_matrix_body_to_ned(
        drone_state.roll,
        drone_state.pitch,
        drone_state.yaw
    )
    R_ned_to_body = R_body_to_ned.T
    target_vec_body = R_ned_to_body @ target_vec_ned

    # Apply FPV camera pitch angle (camera tilted upward)
    # Rotate around Y-axis (right axis) to tilt camera up
    camera_pitch_rad = math.radians(CAMERA_PITCH_ANGLE_DEG)
    cos_cam = math.cos(camera_pitch_rad)
    sin_cam = math.sin(camera_pitch_rad)

    # Rotation matrix for camera pitch (around Y-axis)
    # Positive pitch = camera tilted up = looking upward
    x_forward_cam = target_vec_body[0] * cos_cam + target_vec_body[2] * sin_cam
    y_right_cam = target_vec_body[1]
    z_down_cam = -target_vec_body[0] * sin_cam + target_vec_body[2] * cos_cam

    # Check if target is in front of camera
    if x_forward_cam <= 0.1:  # Behind or too close
        return None

    # Calculate angular position in camera frame
    # angle_x: positive = right (around Z-axis)
    # angle_y: positive = down (around Y-axis)
    distance = math.sqrt(x_forward_cam**2 + y_right_cam**2 + z_down_cam**2)
    angle_x_rad = math.atan2(y_right_cam, x_forward_cam)
    angle_y_rad = math.atan2(z_down_cam, x_forward_cam)

    return angle_x_rad, angle_y_rad, distance

def angular_to_pixel_error(angle_x_rad, angle_y_rad):
    """Convert angular errors to pixel errors for display"""
    pixels_per_rad_x = IMAGE_WIDTH / math.radians(CAMERA_HFOV_DEG)
    pixels_per_rad_y = IMAGE_HEIGHT / math.radians(CAMERA_VFOV_DEG)

    pixel_error_x = angle_x_rad * pixels_per_rad_x
    pixel_error_y = angle_y_rad * pixels_per_rad_y

    return pixel_error_x, pixel_error_y

def calculate_target_bearing(drone_state, target_ned):
    """Calculate bearing from drone to target (0-360°, 0=North)"""
    dn = target_ned[0] - drone_state.pos_n
    de = target_ned[1] - drone_state.pos_e
    bearing_rad = math.atan2(de, dn)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = bearing_deg % 360
    if bearing_deg < 0:
        bearing_deg += 360
    return bearing_deg

def calculate_moving_target_position(center_n, center_e, center_d, elapsed_time):
    """
    Calculate moving target position based on motion type.

    Args:
        center_n, center_e, center_d: Center point for motion (NED frame)
        elapsed_time: Time since test started (seconds)

    Returns:
        np.array([n, e, d]) - target position in NED frame
    """
    if TARGET_MOTION_TYPE == "circle":
        # Circular motion: target moves in a circle around center point
        angular_velocity = TARGET_SPEED / TARGET_CIRCLE_RADIUS  # rad/s
        angle = angular_velocity * elapsed_time

        offset_n = TARGET_CIRCLE_RADIUS * math.cos(angle)
        offset_e = TARGET_CIRCLE_RADIUS * math.sin(angle)

        return np.array([
            center_n + offset_n,
            center_e + offset_e,
            center_d
        ])

    elif TARGET_MOTION_TYPE == "figure8":
        # Figure-8 motion (lemniscate pattern)
        angular_velocity = TARGET_SPEED / TARGET_CIRCLE_RADIUS
        t = angular_velocity * elapsed_time

        # Lemniscate of Gerono parametric equations
        scale = TARGET_CIRCLE_RADIUS
        offset_n = scale * math.sin(t)
        offset_e = scale * math.sin(t) * math.cos(t)

        return np.array([
            center_n + offset_n,
            center_e + offset_e,
            center_d
        ])

    elif TARGET_MOTION_TYPE == "line":
        # Linear motion: target moves back and forth in East-West direction
        period = (2 * TARGET_CIRCLE_RADIUS) / TARGET_SPEED
        t_normalized = (elapsed_time % period) / period  # 0 to 1

        # Triangle wave: 0->1->0
        if t_normalized < 0.5:
            progress = t_normalized * 2  # 0 to 1
        else:
            progress = 2 - (t_normalized * 2)  # 1 to 0

        offset_e = (progress - 0.5) * 2 * TARGET_CIRCLE_RADIUS  # -radius to +radius

        return np.array([
            center_n,
            center_e + offset_e,
            center_d
        ])

    else:
        # Stationary target
        return np.array([center_n, center_e, center_d])


def send_landing_target(mav, angle_x, angle_y, distance):
    """Send LANDING_TARGET message with angular errors"""
    # Calculate target angular size based on distance and person height
    # Person is 5'10" (1.78m) tall
    target_physical_height = TARGET_HEIGHT  # meters (1.78m = 5'10")
    angular_size_rad = math.atan2(target_physical_height, distance) if distance > 0.1 else 0.5

    mav.mav.landing_target_send(
        int(time.time() * 1e6),
        0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        angle_x,
        angle_y,
        0.0,
        angular_size_rad,
        0.0,
        0,  # position valid = 0 (visible)
        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_OTHER
    )

def wait_for_arm(mav, drone_state, timeout=30):
    print("Waiting for ARM...")
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(blocking=True, timeout=1)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                drone_state.update_from_heartbeat(msg)
                if drone_state.armed:
                    print("✓ Armed!")
                    return True
    print("✗ Arm timeout")
    return False

def wait_for_alt(mav, drone_state, target_alt, timeout=30):
    print(f"Waiting for altitude {target_alt}m...")
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(blocking=True, timeout=1)
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'LOCAL_POSITION_NED':
                drone_state.update_from_local_position(msg)
                print(f"  Altitude: {drone_state.altitude:.1f}m", end='\r')
                if drone_state.altitude >= target_alt * 0.95:
                    print(f"\n✓ Reached {drone_state.altitude:.1f}m!")
                    return True
    print("\n✗ Altitude timeout")
    return False

def set_mode(mav, mode_name, mode_num):
    print(f"Setting mode: {mode_name} ({mode_num})...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num,
        0, 0, 0, 0, 0
    )
    time.sleep(1)

def wait_for_position_estimate(mav, timeout=30):
    """Wait for EKF to provide position estimate"""
    print("Waiting for position estimate (GPS/EKF)...")
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if msg:
            # Check if position estimate is good
            flags = msg.flags
            # EKF_POS_HORIZ_ABS flag indicates horizontal position estimate is good
            if flags & 0x01:  # EKF_POS_HORIZ_ABS
                print("✓ Position estimate ready!")
                return True
            print(f"  Waiting... (flags: {flags:04x})", end='\r')

    # Fallback: just wait for GPS fix
    print("\n  Checking GPS...")
    start = time.time()
    while time.time() - start < 10:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if msg and msg.fix_type >= 3:  # 3D fix
            print("✓ GPS 3D fix acquired!")
            time.sleep(2)  # Give EKF time to initialize
            return True

    print("✗ Position estimate timeout")
    return False

def arm_and_takeoff(mav, drone_state, target_alt):
    print("\n" + "=" * 80)
    print("AUTONOMOUS ARM & TAKEOFF SEQUENCE")
    print("=" * 80)

    # Wait for position estimate first
    if not wait_for_position_estimate(mav):
        print("⚠️  No position estimate, but continuing anyway...")
        time.sleep(5)  # Give it more time

    # Set GUIDED mode
    set_mode(mav, "GUIDED", 4)
    time.sleep(2)

    # Arm
    print("\nArming motors...")
    max_arm_attempts = 3
    for attempt in range(max_arm_attempts):
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        if wait_for_arm(mav, drone_state, timeout=10):
            break

        if attempt < max_arm_attempts - 1:
            print(f"  Retry {attempt + 2}/{max_arm_attempts}...")
            time.sleep(2)
    else:
        return False

    time.sleep(2)

    # Takeoff
    print(f"\nTaking off to {target_alt}m...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_alt
    )

    if not wait_for_alt(mav, drone_state, target_alt):
        return False

    time.sleep(2)
    return True

def main():
    print("=" * 80)
    print(" " * 18 + "PIXEL_LOCK MOVING TARGET TEST")
    print("=" * 80)
    print("\nThis test:")
    print(f"  1. Arms and takes off to {TAKEOFF_ALT}m altitude")
    print(f"  2. FPV camera angled {CAMERA_PITCH_ANGLE_DEG}° upward (realistic mount)")
    print(f"  3. Tracks {TARGET_MOTION_TYPE} target at {TARGET_SPEED} m/s")
    print(f"  4. Runs for {TEST_DURATION:.0f} seconds")
    print(f"  5. Verifies aggressive tracking of fast-moving targets")
    print("\n" + "=" * 80)

    # Connect
    print("\nConnecting to ArduPilot...")
    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    print("Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"✓ Connected to system {mav.target_system}\n")

    # Request LOCAL_POSITION_NED messages at 10Hz
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        100000,  # 100ms = 10Hz
        0, 0, 0, 0, 0
    )

    # Initialize state
    drone_state = DroneState()

    # Arm and takeoff
    if not arm_and_takeoff(mav, drone_state, TAKEOFF_ALT):
        print("\n✗ Arm/takeoff failed!")
        return

    # Switch to PIXEL_LOCK
    print("\n" + "=" * 80)
    set_mode(mav, "PIXEL_LOCK", 29)
    time.sleep(1)  # Give mode switch time to settle

    # Request high-rate attitude stream
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
        50,  # 50 Hz
        1    # start
    )

    # Initialize starting position for tracking
    drone_state.initialize_start_position()

    print("✓ PIXEL_LOCK mode active")
    print("=" * 80)

    # Define target center point RELATIVE to drone's current position
    target_center_n = drone_state.pos_n + TARGET_START_OFFSET_N
    target_center_e = drone_state.pos_e + TARGET_START_OFFSET_E
    target_center_d = drone_state.pos_d + TARGET_ALTITUDE_OFFSET

    # Calculate initial heading
    initial_heading = drone_state.get_heading_deg()

    print(f"\n🎯 MOVING TARGET TEST:")
    print(f"   Motion type:    {TARGET_MOTION_TYPE}")
    print(f"   Center point:   N={target_center_n:.1f}m, E={target_center_e:.1f}m, Alt={-target_center_d:.1f}m")
    print(f"   Speed:          {TARGET_SPEED:.1f} m/s")
    if TARGET_MOTION_TYPE in ["circle", "figure8", "line"]:
        print(f"   Radius/Range:   {TARGET_CIRCLE_RADIUS:.1f}m")
    print(f"\n📹 CAMERA CONFIG:")
    print(f"   Pitch angle:    {CAMERA_PITCH_ANGLE_DEG:>6.1f}° (FPV style - tilted upward)")
    print(f"   FOV:            H={CAMERA_HFOV_DEG}° V={CAMERA_VFOV_DEG}°")
    print(f"\n🧭 INITIAL NAVIGATION:")
    print(f"   Drone heading:  {initial_heading:>6.1f}° (0°=N, 90°=E, 180°=S, 270°=W)")
    print("\n" + "-" * 80)

    # Tracking state
    lock_start_time = None
    is_locked = False

    try:
        update_rate = 20  # Hz
        last_print = time.time()
        test_start = time.time()
        iteration = 0

        print(f"\n⏱️  Running test for {TEST_DURATION}s...")
        print("-" * 80 + "\n")

        while time.time() - test_start < TEST_DURATION:
            iteration += 1
            elapsed_time = time.time() - test_start

            # Update drone state from telemetry - process ALL pending messages
            while True:
                msg = mav.recv_match(blocking=False)
                if not msg:
                    break
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    drone_state.update_from_heartbeat(msg)
                elif msg_type == 'ATTITUDE':
                    drone_state.update_from_attitude(msg)
                elif msg_type == 'VFR_HUD':
                    drone_state.update_from_vfr_hud(msg)
                elif msg_type == 'LOCAL_POSITION_NED':
                    drone_state.update_from_local_position(msg)

            # Calculate moving target position
            target_world_ned = calculate_moving_target_position(
                target_center_n, target_center_e, target_center_d, elapsed_time
            )

            # Calculate target position in camera frame
            result = world_point_to_camera_frame(target_world_ned, drone_state)

            if result is not None:
                angle_x, angle_y, distance = result

                # Send LANDING_TARGET
                send_landing_target(mav, angle_x, angle_y, distance)

                # Convert to pixels for display
                px_err_x, px_err_y = angular_to_pixel_error(angle_x, angle_y)

                # Check lock status
                within_threshold = (
                    abs(px_err_x) < LOCK_THRESHOLD_PIXELS_X and
                    abs(px_err_y) < LOCK_THRESHOLD_PIXELS_Y
                )

                if within_threshold:
                    if lock_start_time is None:
                        lock_start_time = time.time()
                    lock_duration = time.time() - lock_start_time

                    if lock_duration >= LOCK_TIME_REQUIRED and not is_locked:
                        is_locked = True
                        print("\n\n" + "=" * 80)
                        print(" " * 30 + "🎯 TARGET LOCKED!")
                        print("=" * 80)
                        print(f"Lock achieved in {lock_duration:.2f}s")
                        print(f"Final error: X={px_err_x:.1f}px, Y={px_err_y:.1f}px")
                        print(f"Holding lock...\n")
                        last_print = time.time()

                    status = "✓ LOCKED" if is_locked else "🎯 LOCKING"
                else:
                    lock_start_time = None
                    if is_locked:
                        is_locked = False
                        print("\n✗ Lost lock!\n")
                    status = "🔍 SEEKING"

                # Print status every 0.5s
                if time.time() - last_print >= 0.5:
                    elapsed = time.time() - test_start
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                    # Calculate navigation info
                    current_heading = drone_state.get_heading_deg()
                    travel_distance = drone_state.get_travel_distance()
                    travel_bearing = drone_state.get_travel_bearing()
                    target_bearing = calculate_target_bearing(drone_state, target_world_ned)

                    print(f"\r[{timestamp}] {status} | "
                          f"Err: X={px_err_x:>6.1f}px Y={px_err_y:>6.1f}px | "
                          f"Drone: N={drone_state.pos_n:>5.1f}m E={drone_state.pos_e:>5.1f}m | "
                          f"Target: N={target_world_ned[0]:>5.1f}m E={target_world_ned[1]:>5.1f}m | "
                          f"Dist={distance:>4.1f}m | T+{elapsed:.1f}s    ",
                          end='', flush=True)
                    last_print = time.time()
            else:
                # Target not visible (behind camera)
                print("\r⚠️  TARGET NOT VISIBLE (behind camera)                    ", end='', flush=True)

            time.sleep(1.0 / update_rate)

        # Test duration complete
        print("\n\n" + "=" * 80)
        print(" " * 25 + "TEST COMPLETE")
        print("=" * 80)

        # Calculate final stats
        total_time = time.time() - test_start
        travel_dist = drone_state.get_travel_distance()
        travel_brg = drone_state.get_travel_bearing()
        final_heading = drone_state.get_heading_deg()
        final_target_bearing = calculate_target_bearing(drone_state, target_world_ned)

        # Calculate distance to target
        dn = target_world_ned[0] - drone_state.pos_n
        de = target_world_ned[1] - drone_state.pos_e
        final_dist_to_target = math.sqrt(dn*dn + de*de)

        print(f"\n⏱️  TIMING:")
        print(f"   Duration: {total_time:.1f}s")

        print(f"\n📍 POSITION:")
        print(f"   Drone start:  N={drone_state.start_n:.1f}m, E={drone_state.start_e:.1f}m")
        print(f"   Drone final:  N={drone_state.pos_n:.1f}m, E={drone_state.pos_e:.1f}m")
        print(f"   Target final: N={target_world_ned[0]:.1f}m, E={target_world_ned[1]:.1f}m")
        print(f"   Target center: N={target_center_n:.1f}m, E={target_center_e:.1f}m")
        print(f"   Motion type:  {TARGET_MOTION_TYPE} @ {TARGET_SPEED:.1f} m/s")

        print(f"\n🧭 NAVIGATION:")
        print(f"   Start heading:  {math.degrees(drone_state.start_yaw):>6.1f}°")
        print(f"   Final heading:  {final_heading:>6.1f}°")
        print(f"   Travel bearing: {travel_brg:>6.1f}° (direction drone moved)")
        print(f"   Target bearing: {final_target_bearing:>6.1f}° (where target is)")

        print(f"\n📏 DISTANCE:")
        print(f"   Distance traveled:  {travel_dist:.2f}m")
        print(f"   Final dist to target: {final_dist_to_target:.2f}m")

        print(f"\n🎯 RESULT:")
        if is_locked:
            print("   ✓ Target locked successfully!")
        else:
            print("   ⚠️  Target not locked")

        print("=" * 80)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print(" " * 32 + "TEST STOPPED")
        print("=" * 80)
        print("\nLanding...")
        set_mode(mav, "LAND", 9)
        time.sleep(5)

if __name__ == '__main__':
    main()
