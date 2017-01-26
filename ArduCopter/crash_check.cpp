#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond angle max is signal we are inverted
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Copter::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // return immediately if disarmed, or crash checking disabled
    if (!motors.armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP) {
        crash_counter = 0;
        return;
    }

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    if (land_accel_ef_filter.get().length() >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control.get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
        // send message to gcs
        gcs_send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming");
        // disarm motors
        init_disarm_motors();
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG  30.0f
#define PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC 0.5f
#define PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS 3.0f
#define PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC 0.5f

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// called at MAIN_LOOP_RATE

void Copter::parachute_check()
{
    // Return immediately if parachute is not enabled
    if (!parachute.enabled()) {
        parachute_check_state.angle_error_excessive = false;
        return;
    }

    // Update the parachute library
    parachute.update();

    // Return immediately if we are disarmed or landed, or the current mode allows angles beyond the tilt limit
    if (!motors.armed() || ap.land_complete) {
        parachute_check_state.angle_error_excessive = false;
        return;
    }

    // Retrieve useful values
    const float angle_error = attitude_control.get_att_error_angle_deg();
    const float tilt_angle = acosf(ahrs.get_rotation_body_to_ned().c.z);
    const float tilt_angle_limit = attitude_control.get_tilt_limit_rad() + radians(PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG);
    const float vel_z = -inertial_nav.get_velocity_z()*0.01f; // Convert from NEU to NED
    const float vel_z_error = -pos_control.get_vel_error_z()*0.01f; // Convert from NEU to NED

    // Check if the angle error is excessive, start the timer when it becomes excessive
    bool new_angle_error_excessive = angle_error > PARACHUTE_ANGLE_ERROR_EXCESSIVE_LIMIT_DEG;
    if (new_angle_error_excessive && !parachute_check_state.angle_error_excessive) {
        parachute_check_state.angle_error_excessive_begin_ms = millis();
    }
    parachute_check_state.angle_error_excessive = new_angle_error_excessive;

    // Timeout occurs if angle error is excessive and PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC has elapsed
    bool angle_error_excessive_timeout = parachute_check_state.angle_error_excessive && (millis()-parachute_check_state.angle_error_excessive_begin_ms)*1e-3f > PARACHUTE_ANGLE_ERROR_EXCESSIVE_TIMEOUT_SEC;

    // Check if the tilt angle is excessive
    bool tilt_angle_excessive = parachute_check_state.angle_error_excessive && tilt_angle > tilt_angle_limit;

    // Check if the there is excessive vertical velocity error, start the timer when it becomes excessive
    bool new_vel_z_error_excessive = pos_control.is_active_z() && fabsf(vel_z) > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS && fabsf(vel_z_error) > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_LIMIT_MPS;
    if (new_vel_z_error_excessive && !parachute_check_state.vel_z_error_excessive) {
        parachute_check_state.vel_z_error_excessive_begin_ms = millis();
    }
    parachute_check_state.vel_z_error_excessive = new_vel_z_error_excessive;

    // Timeout occurs if vertical velocity error is excessive and PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC has elapsed
    bool vel_z_error_excessive_timeout = parachute_check_state.vel_z_error_excessive && (millis()-parachute_check_state.vel_z_error_excessive_begin_ms)*1e-3f > PARACHUTE_VERT_VEL_ERROR_EXCESSIVE_TIMEOUT_SEC;

    // Special case: in stabilize, throttle cut completely, falling
    bool stabilize_throttle_cut = control_mode == STABILIZE && ap.throttle_zero && vel_z > 5.0f;

    if (angle_error_excessive_timeout || tilt_angle_excessive || vel_z_error_excessive_timeout || stabilize_throttle_cut) {
        uint8_t errcode = ERROR_CODE_UNHEALTHY;
        if (angle_error_excessive_timeout) {
            errcode = ERROR_CODE_PARACHUTE_REASON_ANGLE_ERROR_EXCESSIVE_TIMEOUT;
        } else if (tilt_angle_excessive) {
            errcode = ERROR_CODE_PARACHUTE_REASON_TILT_ANGLE_EXCESSIVE;
        } else if (vel_z_error_excessive_timeout) {
            errcode = ERROR_CODE_PARACHUTE_REASON_VEL_Z_ERROR_EXCESSIVE_TIMEOUT;
        } else if (stabilize_throttle_cut) {
            errcode = ERROR_CODE_PARACHUTE_REASON_STABILIZE_THROTTLE_CUT;
        }

        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, errcode);
        parachute_release();
        return;
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
    // send message to gcs and dataflash
    gcs_send_text(MAV_SEVERITY_INFO,"Parachute: Released");
    Log_Write_Event(DATA_PARACHUTE_RELEASED);

    // disarm motors
    init_disarm_motors();

    // release parachute
    parachute.release();

    // deploy landing gear
    landinggear.set_cmd_mode(LandingGear_Deploy);
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
        gcs_send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs_send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
