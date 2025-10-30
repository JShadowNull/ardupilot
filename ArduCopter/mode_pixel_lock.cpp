#include "Copter.h"

/*
 * Init and run calls for PIXEL_LOCK flight mode
 *
 * PIXEL_LOCK mode uses vision-based pixel error to control velocity
 * Similar to GUIDED mode but uses camera feedback instead of GPS
 */

// Control gains and parameters are now configurable via PLCK_* parameters in g2
// Jetson sends: angle_x/angle_y = bbox center position (radians)
//               target_size_y = bbox height (radians)

// Apply deadband to error value
float ModePixelLock::apply_deadband(float error, float deadband) const
{
    if (fabsf(error) < deadband) {
        return 0.0f;
    }
    return (error > 0.0f) ? (error - deadband) : (error + deadband);
}

// pixel_lock_init - initialise PIXEL_LOCK controller
bool ModePixelLock::init(bool ignore_checks)
{
    // Set horizontal speed and acceleration limits (convert m/s to cm/s)
    float max_speed_xy_cms = copter.g2.pixel_lock_max_vel * 100.0f;
    float max_accel_xy_cmss = 100.0f;  // 1 m/s^2 default acceleration
    pos_control->set_max_speed_accel_xy(max_speed_xy_cms, max_accel_xy_cmss);
    pos_control->set_correction_speed_accel_xy(max_speed_xy_cms, max_accel_xy_cmss);

    // Set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);

    // Initialize horizontal position controller
    pos_control->init_xy_controller();

    // Initialize vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    return true;
}

// pixel_lock_run - runs the PIXEL_LOCK controller
// should be called at 400hz
void ModePixelLock::run()
{
    // if not armed or landed, make safe
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // Set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Check for vision timeout (using parameter)
    uint32_t time_since_update = AP_HAL::millis() - copter.pixel_tracker.last_update_ms;
    bool vision_valid = (time_since_update < (uint32_t)copter.g2.pixel_lock_timeout) && copter.pixel_tracker.visible;

    // Default to zero velocity
    Vector2f vel_target_ne_ms;
    vel_target_ne_ms.zero();
    float climb_rate_ms = 0.0f;
    float yaw_rate_rads = 0.0f;

    if (vision_valid) {
        // Get angular errors from pixel tracker (radians)
        float angular_error_x = copter.pixel_tracker.angle_error_x;  // Positive = target right
        float angular_error_y = copter.pixel_tracker.angle_error_y;  // Positive = target down

        // Apply deadband (convert parameter from degrees to radians)
        float deadband_rad = radians(copter.g2.pixel_lock_deadband);
        angular_error_x = apply_deadband(angular_error_x, deadband_rad);
        angular_error_y = apply_deadband(angular_error_y, deadband_rad);

        // MULTI-ZONE CONTROL STRATEGY:
        // FAR ZONE (>PLCK_CLOSE_DIST): Aggressive pursuit - max speed, high gains
        // CLOSE ZONE (<PLCK_CLOSE_DIST): Precision tracking - reduced speed, lower gains
        //
        // This prevents overshoot while maintaining fast initial approach.
        // Gain scheduling smoothly transitions between zones.

        // Estimate distance to target (for ground targets: distance ≈ altitude)
        float estimated_distance = -copter.inertial_nav.get_position_z_up_cm() * 0.01f;  // cm to m
        if (estimated_distance < 5.0f) {
            estimated_distance = 5.0f;  // Minimum distance
        }

        // Get target size for distance control
        float current_size = copter.pixel_tracker.target_size_y;
        float target_size_rad = radians(copter.g2.pixel_lock_target_size);
        float size_error = current_size - target_size_rad;
        float size_deadband_rad = radians(copter.g2.pixel_lock_size_deadband);
        size_error = apply_deadband(size_error, size_deadband_rad);

        // GAIN SCHEDULING: Smoothly transition between far and close modes
        // Use linear interpolation based on distance
        float close_dist = copter.g2.pixel_lock_close_dist;
        float transition_range = 5.0f;  // 5m transition zone for smooth blending
        float blend_factor;  // 0.0 = full close mode, 1.0 = full far mode

        if (estimated_distance <= close_dist) {
            blend_factor = 0.0f;  // Full close-range mode
        } else if (estimated_distance >= close_dist + transition_range) {
            blend_factor = 1.0f;  // Full far-range mode
        } else {
            // Smooth transition zone
            blend_factor = (estimated_distance - close_dist) / transition_range;
        }

        // Interpolate gains and limits based on distance
        float active_vel_gain = copter.g2.pixel_lock_close_gain +
                                blend_factor * (copter.g2.pixel_lock_vel_gain - copter.g2.pixel_lock_close_gain);
        float active_max_vel = copter.g2.pixel_lock_close_max_vel +
                               blend_factor * (copter.g2.pixel_lock_max_vel - copter.g2.pixel_lock_close_max_vel);
        float active_yaw_gain = 2.0f + blend_factor * 3.0f;  // 2.0 close, 5.0 far

        // 1. YAW CONTROL - Point at target (primary horizontal centering)
        // angular_error_x: positive = target right of center = yaw right
        yaw_rate_rads = angular_error_x * active_yaw_gain;
        yaw_rate_rads = constrain_float(yaw_rate_rads, radians(-180.0f), radians(180.0f));

        // 2. FORWARD VELOCITY - Distance-adaptive approach
        float vel_forward_ms = 0.0f;
        float abs_angular_error_x = fabsf(angular_error_x);

        if (blend_factor > 0.5f) {
            // FAR MODE: Aggressive approach
            // Base forward velocity on size error (distance control)
            vel_forward_ms = -size_error * active_vel_gain * 3.0f;

            // If not centered, ADD extra forward velocity to catch up while yawing
            if (abs_angular_error_x > radians(10.0f)) {
                vel_forward_ms += active_max_vel * 0.5f;
            }
        } else {
            // CLOSE MODE: Precision approach - no extra boost, pure size control
            vel_forward_ms = -size_error * active_vel_gain;

            // Reduce velocity if significantly off-center to prevent overshoot
            if (abs_angular_error_x > radians(15.0f)) {
                vel_forward_ms *= 0.5f;  // Cut velocity in half when misaligned
            }
        }

        // Constrain to active max velocity
        vel_forward_ms = constrain_float(vel_forward_ms, -active_max_vel, active_max_vel);

        // Convert forward velocity to NED frame (no lateral component!)
        float yaw_rad = ahrs.get_yaw();
        vel_target_ne_ms.x = vel_forward_ms * cosf(yaw_rad);  // North
        vel_target_ne_ms.y = vel_forward_ms * sinf(yaw_rad);  // East

        // 3. ALTITUDE control - Coordinate with pitch to keep target in frame
        // KEY INSIGHT: When we pitch forward aggressively (high vel_forward_ms),
        // camera tilts down and target appears higher in frame.
        // Compensate by reducing climb rate when pitching forward.

        // Base altitude correction on vertical error
        float base_climb_rate = -angular_error_y * estimated_distance * active_vel_gain;

        // Pitch compensation: Estimate pitch angle from forward velocity
        // Higher forward velocity = more pitch = more downward camera tilt
        // This requires LESS climb to keep target centered vertically
        float pitch_compensation_factor = 1.0f;
        if (fabsf(vel_forward_ms) > 1.0f) {
            // Reduce altitude correction when moving fast forward
            // At 10 m/s: reduce by 50%, at 5 m/s: reduce by 25%
            pitch_compensation_factor = 1.0f - (fabsf(vel_forward_ms) / active_max_vel) * 0.5f;
        }

        climb_rate_ms = base_climb_rate * pitch_compensation_factor;

        // Use distance-scaled climb limit (gentler when close)
        float active_max_climb = copter.g2.pixel_lock_max_climb * (0.5f + 0.5f * blend_factor);  // 50%-100% based on distance
        climb_rate_ms = constrain_float(climb_rate_ms, -active_max_climb, active_max_climb);
    }

    // Send velocity commands to position controller (convert m/s to cm/s)
    Vector2f vel_target_cms;
    vel_target_cms.x = vel_target_ne_ms.x * 100.0f;  // North (cm/s)
    vel_target_cms.y = vel_target_ne_ms.y * 100.0f;  // East (cm/s)
    Vector2f accel_target;
    accel_target.zero();
    pos_control->input_vel_accel_xy(vel_target_cms, accel_target);

    // Send climb rate to altitude controller (convert m/s to cm/s)
    float climb_rate_cms = climb_rate_ms * 100.0f;
    float accel_z_cmss = 0.0f;
    pos_control->input_vel_accel_z(climb_rate_cms, accel_z_cmss, false);

    // Run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // Apply attitude control with yaw rate (convert radians to degrees)
    float yaw_rate_degs = degrees(yaw_rate_rads);
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), yaw_rate_degs);
}
