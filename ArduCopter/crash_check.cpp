#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond angle max is signal we are inverted
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// Code to detect a thrust loss main ArduCopter code
#define THRUST_LOSS_CHECK_TRIGGER_SEC         1     // 1 second descent while level and high throttle indicates thrust loss
#define THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD  1500  // we can't expect to maintain altitude beyond 15 degrees on all aircraft
#define THRUST_LOSS_CHECK_MINIMUM_THROTTLE    0.9f  // we can expect to maintain altitude above 90 % throttle

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Copter::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // return immediately if disarmed, or crash checking disabled
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        crash_counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == Mode::Number::ACRO || control_mode == Mode::Number::FLIP) {
        crash_counter = 0;
        return;
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    //return immediately if in autorotation mode
    if (control_mode == Mode::Number::AUTOROTATE) {
        crash_counter = 0;
        return;
    }
#endif

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    if (land_accel_ef_filter.get().length() >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);
        // keep logging even if disarmed:
        AP::logger().set_force_log_disarmed(true);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming");
        // disarm motors
        copter.arming.disarm();
    }
}

// check for loss of thrust and trigger thrust boost in motors library
void Copter::thrust_loss_check()
{
    static uint16_t thrust_loss_counter;  // number of iterations vehicle may have been crashed

    // exit immediately if thrust boost is already engaged
    if (motors->get_thrust_boost()) {
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        thrust_loss_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // check for desired angle over 15 degrees
    // todo: add thrust angle to AC_AttitudeControl
    const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
    if (sq(angle_target.x) + sq(angle_target.y) > sq(THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for throttle over 90% or throttle saturation
    if ((attitude_control->get_throttle_in() < THRUST_LOSS_CHECK_MINIMUM_THROTTLE) && (!motors->limit.throttle_upper)) {
        thrust_loss_counter = 0;
        return;
    }

    // check throttle is over 25% to prevent checks triggering from thrust limitations caused by low commanded throttle
    if ((attitude_control->get_throttle_in() < 0.25f)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for descent
    if (!is_negative(inertial_nav.get_velocity_z())) {
        thrust_loss_counter = 0;
        return;
    }

    // check for angle error over 30 degrees to ensure the aircraft has attitude control
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error >= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        thrust_loss_counter = 0;
        return;
    }

    // the aircraft is descending with low requested roll and pitch, at full available throttle, with attitude control
    // we may have lost thrust
    thrust_loss_counter++;

    // check if thrust loss for 1 second
    if (thrust_loss_counter >= (THRUST_LOSS_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // reset counter
        thrust_loss_counter = 0;
        AP::logger().Write_Error(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)motors->get_lost_motor() + 1);
        // enable thrust loss handling
        motors->set_thrust_boost(true);
        // the motors library disables this when it is no longer needed to achieve the commanded output
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG  30.0f
#define PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC 0.5f
#define PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS 3.0f
#define PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC 0.5f

// parachute_check - checks whether the parachute should be deployed. There are several criteria for parachute deployment. The parachute will deploy when:
// - Attitude error is greater than PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG for PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC seconds
// - Attitude error is greater than PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG *AND* tilt angle is greater than ANGLE_MAX + PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG
// - Height control is active *AND* vertical velocity error is above PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS *AND* abs(vertical velocity) is above PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS *AND* all of these conditions are true for PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC
// - Flight mode is STABILIZE *AND* throttle is zero *AND* downward velocity is greater than 5 m/s
// called at MAIN_LOOP_RATE

void Copter::parachute_check()
{
    // Return immediately if parachute is not enabled
    if (!parachute.enabled()) {
        parachute_check_state.angle_error_excessive = false;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == Mode::Number::ACRO || control_mode == Mode::Number::FLIP) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying
    if (ap.land_complete) {
        control_loss_count = 0;
        parachute_check_state.angle_error_excessive = false;
        return;
    }

    // Retrieve useful values
    const float angle_error = attitude_control->get_att_error_angle_deg();
    const float tilt_angle = acosf(ahrs.get_rotation_body_to_ned().c.z);
    const float tilt_angle_limit = attitude_control->get_tilt_limit_rad() + radians(PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG);
    const float vel_z = -inertial_nav.get_velocity_z()*0.01f; // Convert cm/s to m/s and convert NEU to NED
    const float vel_z_error = -pos_control->get_vel_error_z()*0.01f; // Convert cm/s to m/s and convert NEU to NED

    // Start attitude error timer
    bool new_angle_error_excessive = angle_error > PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG;
    if (new_angle_error_excessive && !parachute_check_state.angle_error_excessive) {
        parachute_check_state.angle_error_excessive_begin_ms = millis();
    }
    parachute_check_state.angle_error_excessive = new_angle_error_excessive;

    // Start vertical velocity error timer
    bool new_vel_z_error_excessive = pos_control->is_active_z() && fabsf(vel_z) > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS && fabsf(vel_z_error) > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS;
    if (new_vel_z_error_excessive && !parachute_check_state.vel_z_error_excessive) {
        parachute_check_state.vel_z_error_excessive_begin_ms = millis();
    }
    parachute_check_state.vel_z_error_excessive = new_vel_z_error_excessive;

    // Check for criterion: "Attitude error is greater than PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG for PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC seconds"
    bool angle_error_excessive_timeout = parachute_check_state.angle_error_excessive && (millis()-parachute_check_state.angle_error_excessive_begin_ms)*1e-3f > PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC;

    // Check for criterion: "Attitude error is greater than PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG *AND* tilt angle is greater than ANGLE_MAX + PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG"
    bool tilt_angle_excessive = parachute_check_state.angle_error_excessive && tilt_angle > tilt_angle_limit;

    // Check for criterion: "Height control is active *AND* vertical velocity error is above PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS *AND* abs(vertical velocity) is above PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS *AND* all of these conditions are true for PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC"
    bool vel_z_error_excessive_timeout = parachute_check_state.vel_z_error_excessive && (millis()-parachute_check_state.vel_z_error_excessive_begin_ms)*1e-3f > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC;

    // Check for criterion: "Flight mode is STABILIZE *AND* throttle is zero *AND* downward velocity is greater than 5 m/s"
    bool stabilize_throttle_cut = control_mode == STABILIZE && ap.throttle_zero && vel_z > 5.0f;

    // Deploy the parachute if a criterion is met
    if (angle_error_excessive_timeout) {
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_REASON_ANGLE_ERROR_EXCESSIVE_TIMEOUT);
        parachute_release();
    } else if (tilt_angle_excessive) {
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_REASON_TILT_ANGLE_EXCESSIVE);
        parachute_release();
    } else if (vel_z_error_excessive_timeout) {
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_REASON_VEL_Z_ERROR_EXCESSIVE_TIMEOUT);
        parachute_release();
    } else if (stabilize_throttle_cut) {
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_REASON_STABILIZE_THROTTLE_CUT);
        parachute_release();
    }

    // pass sink rate to parachute library
    parachute.set_sink_rate(-inertial_nav.get_velocity_z() * 0.01);
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
    // disarm motors
    arming.disarm();

    // release parachute
    parachute.release();

    // deploy landing gear
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
void Copter::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if vehicle is landed
    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
#if 0
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_TOO_LOW);
        return;
    }
#endif

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
