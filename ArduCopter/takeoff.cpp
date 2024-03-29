#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (!copter.motors->armed()) {
        return false;
    }
    if (!copter.ap.land_complete) {
        // can't takeoff again!
        return false;
    }
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
    if (takeoff_alt_cm <= copter.current_loc.alt) {
        // can't takeoff downwards...
        return false;
    }

    // Helicopters should return false if MAVlink takeoff command is received while the rotor is not spinning
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED && copter.ap.using_interlock) {
        return false;
    }

    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in centimeters
void Mode::_TakeOff::start(float alt_cm)
{
    // indicate we are taking off
    copter.set_land_complete(false);
    // tell position controller to reset alt target and reset I terms
    copter.set_throttle_takeoff();

    // calculate climb rate
    const float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));

    // sanity check speed and target
    if (speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    _running = true;
    max_speed = speed;
    start_ms = millis();
    alt_delta = alt_cm;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
    start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void Mode::_TakeOff::get_climb_rates(float& pilot_climb_rate,
                                                  float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // acceleration of 50cm/s/s
    static constexpr float TAKEOFF_ACCEL = 50.0f;
    const float takeoff_minspeed = MIN(50.0f, max_speed);
    const float time_elapsed = (millis() - start_ms) * 1.0e-3f;
    const float speed = MIN(time_elapsed * TAKEOFF_ACCEL + takeoff_minspeed, max_speed);

    const float time_to_max_speed = (max_speed - takeoff_minspeed) / TAKEOFF_ACCEL;
    float height_gained;
    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_elapsed) + takeoff_minspeed * time_elapsed;
    } else {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_to_max_speed) + takeoff_minspeed * time_to_max_speed +
                        (time_elapsed - time_to_max_speed) * max_speed;
    }

    // check if the takeoff is over
    if (height_gained >= alt_delta) {
        stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0.0f;
        } else {
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) {
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate;
        } else {
            pilot_climb_rate = 0.0f;
        }
    }
}

void Mode::auto_takeoff_set_start_alt(void)
{
    // start with our current altitude
    auto_takeoff_no_nav_alt_cm = inertial_nav.get_altitude();
    auto_takeoff_max_nav_alt_cm = inertial_nav.get_altitude();

    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        // we are not flying, add the wp_navalt_min
        auto_takeoff_no_nav_alt_cm += g2.wp_navalt_min * 100;
        auto_takeoff_max_nav_alt_cm += g2.wp_navalt_max * 100;
        auto_takeoff_max_nav_alt_cm = MAX(auto_takeoff_max_nav_alt_cm,auto_takeoff_no_nav_alt_cm);
    }

    // reset the min baro alt seen in takeoff
    copter.takeoff_baro_min_alt_m = copter.barometer.get_altitude();
    copter.takeoff_liftoff_complete = false;
}


/*
  call attitude controller for automatic takeoff, limiting roll/pitch
  if below wp_navalt_min
 */
void Mode::auto_takeoff_attitude_run(float target_yaw_rate)
{
    float nav_roll, nav_pitch;

    nav_roll = wp_nav->get_roll();
    nav_pitch = wp_nav->get_pitch();

    /*
      record the minimum barometer altitude we see in the
      takeoff. This is used to detect the "dip" that happens on the
      baro when the motors start applying thrust.
     */
    float barometer_alt = copter.barometer.get_altitude();
    if (barometer_alt < copter.takeoff_baro_min_alt_m) {
        copter.takeoff_baro_min_alt_m = barometer_alt;
    }

    /*
      check conditions for liftoff completion
      We consider liftoff to be completed under one of 3 conditions:
        1) the TOFF_BARO_DIP and TOFF_HOV_PCT are both zero
        2) the barometric height has risen from the minimum by at least TOFF_BARO_DIP
        2) the throttle has reached TOFF_HOV_PCT of the hover throttle
     */
    if (!copter.takeoff_liftoff_complete) {
        if (copter.matternet.tkoff_baro_dip > 0 &&
            barometer_alt - copter.takeoff_baro_min_alt_m >= copter.matternet.tkoff_baro_dip) {
            copter.takeoff_liftoff_complete = true;
            gcs().send_text(MAV_SEVERITY_INFO, "LIFTOFF: baro %.1f thr=%.2f\n", barometer_alt - copter.takeoff_baro_min_alt_m, motors->get_throttle());
        }
        if (copter.matternet.tkoff_hover_pct > 0 &&
            motors->get_throttle() >= copter.matternet.tkoff_hover_pct * 0.01 * motors->get_throttle_hover()) {
            copter.takeoff_liftoff_complete = true;
            gcs().send_text(MAV_SEVERITY_INFO, "LIFTOFF: throttle %.2f baro=%.1f\n", motors->get_throttle(), barometer_alt - copter.takeoff_baro_min_alt_m);
        }
        if (copter.matternet.tkoff_baro_dip <= 0 &&
            copter.matternet.tkoff_hover_pct <= 0) {
            // both are disabled, do direct takeoff
            copter.takeoff_liftoff_complete = true;
        }
    }


    float alt_cm = inertial_nav.get_altitude();
    if (!copter.takeoff_liftoff_complete) {
        // below no nav alt we zero roll and pitch demand
        nav_roll = 0;
        nav_pitch = 0;
        // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
        pos_control->set_limit_accel_xy();

        // clear rate integrators till we reach min navigation altitude
        attitude_control->get_rate_roll_pid().set_integrator(0);
        attitude_control->get_rate_pitch_pid().set_integrator(0);
        attitude_control->get_rate_yaw_pid().set_integrator(0);

        wp_nav->shift_wp_origin_to_current_pos();

        auto_takeoff_max_nav_alt_cm = alt_cm + g2.wp_navalt_max * 100;
    } else if (g2.wp_navalt_max > 0 && alt_cm < auto_takeoff_max_nav_alt_cm) {
        // between no nav alt and max nav alt we interpolate
        // roll/pitch demand
        float lean_limit = linear_interpolate(0, copter.aparm.angle_max,
                                              alt_cm,
                                              auto_takeoff_no_nav_alt_cm, auto_takeoff_max_nav_alt_cm);
        if (fabsf(nav_roll) > lean_limit ||
            fabsf(nav_pitch) > lean_limit) {
            nav_roll = constrain_float(nav_roll, -lean_limit, lean_limit);
            nav_pitch = constrain_float(nav_pitch, -lean_limit, lean_limit);
            // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
            pos_control->set_limit_accel_xy();
        }
    }
    
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    if (copter.ap.land_complete) {
        return false;
    }
    if (takeoff.running()) {
        return true;
    }
    return false;
}
