#include "Copter.h"

/*
 * Init and run calls for PIXEL_LOCK flight mode
 *
 * PIXEL_LOCK mode: ViSP IBVS Velocity Control
 *
 * - Accepts computed velocity commands from companion computer (Jetson + ViSP)
 * - Uses full 4-corner IBVS with computed Jacobian for precise control
 * - Velocity commands via SET_POSITION_TARGET_LOCAL_NED MAVLink message
 * - All visual servoing computation (Jacobian, control law) done on Jetson
 * - ArduPilot executes smooth velocity tracking via position controller
 *
 * Architecture:
 *   Jetson: Camera → YOLO → 4 Corners → ViSP (Jacobian + IBVS) → Velocities
 *   MAVLink: SET_POSITION_TARGET_LOCAL_NED (velocity commands)
 *   ArduPilot: Velocity → Position Controller → Smooth Motion
 *
 * Input: Jetson sends via SET_POSITION_TARGET_LOCAL_NED message:
 *   vx, vy, vz = velocity in NED frame (m/s)
 *   yaw_rate = yaw rate (rad/s)
 *   type_mask = 0x0FC7 (velocity + yaw_rate only)
 */

// pixel_lock_init - initialise PIXEL_LOCK controller
bool ModePixelLock::init(bool ignore_checks)
{
    // Initialize mode - no additional setup required
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

    // Check for velocity command timeout
    uint32_t time_since_velocity_update = AP_HAL::millis() - copter.pixel_tracker.velocity_update_ms;
    bool velocity_commands_valid = (time_since_velocity_update < (uint32_t)copter.g2.pixel_lock_vel_timeout);

    // Hover in place if no valid velocity commands received
    float yaw_rate_degs = 0.0f;

    if (velocity_commands_valid && copter.pixel_tracker.use_velocity_control) {
        // ================================================================
        // ViSP IBVS VELOCITY CONTROL
        // ================================================================
        // Use externally computed velocities from companion computer (Jetson + ViSP)
        // These velocities come from full 4-corner IBVS with computed Jacobian

        // Get commanded velocities (already in NED frame, cm/s)
        Vector3f vel_ned_cms = copter.pixel_tracker.velocity_ned_cms;
        float yaw_rate_rads = copter.pixel_tracker.yaw_rate_rads;

        // Use input_vel_accel API for pure velocity control (like GUIDED mode)
        // This prevents position controller from adding position error corrections
        Vector3f accel_target_cmss(0.0f, 0.0f, 0.0f);  // Zero acceleration (velocity-only control)
        pos_control->input_vel_accel_xy(vel_ned_cms.xy(), accel_target_cmss.xy(), false);
        pos_control->input_vel_accel_z(vel_ned_cms.z, accel_target_cmss.z, false);

        // Disable position error integration (critical for pure velocity control)
        pos_control->stop_vel_xy_stabilisation();

        // Set yaw rate via auto_yaw (prevents position controller from overriding yaw)
        yaw_rate_degs = degrees(yaw_rate_rads);
        auto_yaw.set_rate(yaw_rate_degs);

#if HAL_LOGGING_ENABLED
        // Log velocity commands for debugging (use GUIDED logging format for compatibility)
        // This logs as ModeGuided::SubMode::VelAccel type for analysis tools
        copter.Log_Write_Guided_Position_Target(ModeGuided::SubMode::VelAccel, Vector3f(), false, vel_ned_cms, accel_target_cmss);
#endif
    } else {
        // No valid velocity commands - hover in place
        Vector3f vel_zero(0.0f, 0.0f, 0.0f);
        Vector3f accel_zero(0.0f, 0.0f, 0.0f);
        pos_control->input_vel_accel_xy(vel_zero.xy(), accel_zero.xy(), false);
        pos_control->input_vel_accel_z(vel_zero.z, accel_zero.z, false);
        auto_yaw.set_rate(0.0f);
    }

    // ================================================================
    // Apply velocity commands via position controller
    // ================================================================

    // Update XY position controller (processes velocity targets and computes roll/pitch)
    pos_control->update_xy_controller();

    // Update Z controller
    pos_control->update_z_controller();

    // Apply thrust vector from position controller with yaw from auto_yaw
    // This prevents position controller's automatic yaw-to-velocity from interfering
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(),
        auto_yaw.get_heading()
    );
}
