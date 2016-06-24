/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO("ENABLED", 0, AC_PrecLand, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav) :
    _ahrs(ahrs),
    _inav(inav),
    _last_update_ms(0),
    _backend(NULL)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    _attitude_history.push_back(_ahrs.get_rotation_body_to_ned());
    
    // run backend update
    if (_backend != NULL && _enabled) {
        // read from sensor
        _backend->update();

        Vector3f vehicleVelocityNED = _inav.get_velocity()*0.01f;
        vehicleVelocityNED.z = -vehicleVelocityNED.z;

        if (target_acquired()) {
            // EKF prediction step
            float dt = _ahrs.get_ins().get_delta_velocity_dt();
            Vector3f targetDelVel;
            _ahrs.get_ins().get_delta_velocity(targetDelVel);
            targetDelVel = _ahrs.get_rotation_body_to_ned() * _ahrs.get_rotation_autopilot_body_to_vehicle_body() * targetDelVel;
            targetDelVel.z += GRAVITY_MSS*dt;
            targetDelVel = -targetDelVel;

            _ekf_x.predict(dt, targetDelVel.x, 0.5f*dt);
            _ekf_y.predict(dt, targetDelVel.y, 0.5f*dt);

            if (_inav.get_filter_status().flags.horiz_pos_rel) {
                _ekf_x.fuseVel(-vehicleVelocityNED.x, sq(1.0f));
                _ekf_y.fuseVel(-vehicleVelocityNED.y, sq(1.0f));
            }
        }

        if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
            // we have a new, unique los measurement
            _last_backend_los_meas_ms = _backend->los_meas_time_ms();

            Vector3f target_vec_unit_body;
            _backend->get_los_body(target_vec_unit_body);
            Vector3f target_vec_unit_ned = _attitude_history.front()*target_vec_unit_body;

            bool target_vec_valid = target_vec_unit_ned.z > 0.0f;

            if (target_vec_valid && rangefinder_alt_valid && rangefinder_alt_cm > 0.0f) {
                float alt = MAX(rangefinder_alt_cm*0.01f, 0.0f);
                float dist = alt/target_vec_unit_ned.z;
                Vector3f targetPosRelMeasNED = Vector3f(target_vec_unit_ned.x*dist, target_vec_unit_ned.y*dist, alt);

                float xy_pos_var = sq(targetPosRelMeasNED.z*(0.01f + 0.01f*_ahrs.get_gyro().length()) + 0.02f);
                if (!target_acquired()) {
                    // reset filter state
                    if (_inav.get_filter_status().flags.horiz_pos_rel) {
                        _ekf_x.init(targetPosRelMeasNED.x, xy_pos_var, -vehicleVelocityNED.x, sq(1.0f));
                        _ekf_y.init(targetPosRelMeasNED.y, xy_pos_var, -vehicleVelocityNED.y, sq(1.0f));
                    } else {
                        _ekf_x.init(targetPosRelMeasNED.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(targetPosRelMeasNED.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                } else {
                    float NIS_x = _ekf_x.getPosNIS(targetPosRelMeasNED.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(targetPosRelMeasNED.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(targetPosRelMeasNED.x, xy_pos_var);
                        _ekf_y.fusePos(targetPosRelMeasNED.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }
        }
    }
}

bool AC_PrecLand::target_acquired() const
{
    return (AP_HAL::millis()-_last_update_ms) < 2000;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    ret.x = _ekf_x.getPos()*100.0f + _inav.get_position().x;
    ret.y = _ekf_y.getPos()*100.0f + _inav.get_position().y;
    return true;
}

bool AC_PrecLand::get_target_position_relative_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    ret.x = _ekf_x.getPos()*100.0f;
    ret.y = _ekf_y.getPos()*100.0f;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative_cms(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }
    ret.x = _ekf_x.getVel()*100.0f;
    ret.y = _ekf_y.getVel()*100.0f;
    return true;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != NULL) {
        _backend->handle_msg(msg);
    }
}

// send landing target mavlink message to ground station
void AC_PrecLand::send_landing_target(mavlink_channel_t chan) const
{
    mavlink_msg_landing_target_send(chan,
        _last_update_ms,
        target_acquired(),
        MAV_FRAME_GLOBAL_TERRAIN_ALT,
        _ekf_x.getPos()*100.0f, _ekf_y.getPos()*100.0f, 0.0f, 0.0f, 0.0f);

}
