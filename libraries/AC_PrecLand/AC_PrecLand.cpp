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
void AC_PrecLand::update(float alt_above_terrain_cm)
{
    _attitude_history.push_back(_ahrs.get_rotation_body_to_ned());
    
    // run backend update
    if (_backend != NULL && _enabled) {
        // read from sensor
        _backend->update();

        bool prev_target_acquired = target_acquired();
        float xy_pos_var = sq(_target_pos_rel.z*0.01f*0.01f);

        if (target_acquired()) {
            // EKF prediction step
            float dt = _ahrs.get_ins().get_delta_velocity_dt();
            Vector3f targetDelVel;
            _ahrs.get_ins().get_delta_velocity(targetDelVel);
            targetDelVel = _ahrs.get_rotation_body_to_ned()*targetDelVel;
            targetDelVel.z += GRAVITY_MSS*dt;
            targetDelVel = -targetDelVel;

            _vel_ekf_x.predict(dt, targetDelVel.x, 0.5f*dt);
            _vel_ekf_y.predict(dt, targetDelVel.y, 0.5f*dt);

            if (_inav.get_filter_status().flags.horiz_pos_rel) {
                _vel_ekf_x.fuseVel(-_inav.get_velocity().x*0.01f, sq(0.5f));
                _vel_ekf_y.fuseVel(-_inav.get_velocity().y*0.01f, sq(0.5f));
            }
        }
        
        if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
            // we have a new, unique los measurement
            _last_backend_los_meas_ms = _backend->los_meas_time_ms();

            Vector3f target_vec_unit_body;
            _backend->get_los_body(target_vec_unit_body);
            
            calc_angles_and_pos(target_vec_unit_body, alt_above_terrain_cm);

            _vel_ekf_x.fusePos(_target_pos_rel.x*0.01f, xy_pos_var);
            _vel_ekf_y.fusePos(_target_pos_rel.y*0.01f, xy_pos_var);
        }

        if (target_acquired()) {
            if (!prev_target_acquired) {
                // reset filter state
                if (_inav.get_filter_status().flags.horiz_pos_rel) {
                    _vel_ekf_x.init(_target_pos_rel.x*0.01f, xy_pos_var, -_inav.get_velocity().x*0.01f, sq(0.5f));
                    _vel_ekf_y.init(_target_pos_rel.y*0.01f, xy_pos_var, -_inav.get_velocity().y*0.01f, sq(0.5f));
                } else {
                    _vel_ekf_x.init(_target_pos_rel.x*0.01f, xy_pos_var, 0.0f, sq(10.0f));
                    _vel_ekf_y.init(_target_pos_rel.y*0.01f, xy_pos_var, 0.0f, sq(10.0f));
                }
            }

            _target_vel_rel.x = _vel_ekf_x.getVel()*100.0f;
            _target_vel_rel.y = _vel_ekf_y.getVel()*100.0f;
            _target_vel_rel.z = -_inav.get_velocity().z;
        }
    }
}

bool AC_PrecLand::target_acquired()
{
    return (AP_HAL::millis()-_last_update_ms) < 1000;
}

bool AC_PrecLand::get_target_position(Vector3f& ret)
{
    if (!target_acquired()) {
        return false;
    }

    ret = _target_pos;
    return true;
}

bool AC_PrecLand::get_target_position_relative(Vector3f& ret)
{
    if (!target_acquired()) {
        return false;
    }

    ret = _target_pos_rel;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative(Vector3f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_vel_rel;
    return true;
}

// converts sensor's body-frame angles to earth-frame angles and position estimate
//  raw sensor angles stored in _angle_to_target (might be in earth frame, or maybe body frame)
//  earth-frame angles stored in _ef_angle_to_target
//  position estimate is stored in _target_pos
void AC_PrecLand::calc_angles_and_pos(const Vector3f& target_vec_unit_body, float alt_above_terrain_cm)
{
    // rotate into NED frame
    Vector3f target_vec_unit_ned = _attitude_history.front()*target_vec_unit_body;

    // extract the angles to target (logging only)
    _angle_to_target.x = atan2f(-target_vec_unit_body.y, target_vec_unit_body.z);
    _angle_to_target.y = atan2f( target_vec_unit_body.x, target_vec_unit_body.z);
    _ef_angle_to_target.x = atan2f(-target_vec_unit_ned.y, target_vec_unit_ned.z);
    _ef_angle_to_target.y = atan2f( target_vec_unit_ned.x, target_vec_unit_ned.z);

    if (target_vec_unit_ned.z > 0.0f) {
        // get current altitude (constrained to be positive)
        float alt = MAX(alt_above_terrain_cm, 0.0f);
        float dist = alt/target_vec_unit_ned.z;
        _target_pos_rel.x = target_vec_unit_ned.x*dist;
        _target_pos_rel.y = target_vec_unit_ned.y*dist;
        _target_pos_rel.z = alt;  // not used
        _target_pos = _inav.get_position()+_target_pos_rel;

        _last_update_ms = AP_HAL::millis();
    }
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
        AP_HAL::millis()-_last_update_ms < 1000,
        MAV_FRAME_GLOBAL_TERRAIN_ALT,
        _ef_angle_to_target.x, _ef_angle_to_target.y, _target_pos_rel.z, 0.0, 0.0);

}
