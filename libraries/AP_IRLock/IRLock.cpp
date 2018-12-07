
/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// retrieve body frame x and y angles (in radians) to target
// returns true if data is available
bool IRLock::get_angle_to_target_rad(float &x_angle_rad, float &y_angle_rad) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }

    // use data from first (largest) object
    x_angle_rad = atanf(_target_info[0].pos_x);
    y_angle_rad = atanf(_target_info[0].pos_y);
    return true;
}

// retrieve body frame unit vector in direction of target
// returns true if data is available
bool IRLock::get_unit_vector_body(Vector3f& ret) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }

    //Here there shold be aa check that cmpares the timestamp at which the previous velue of count was plural
    //And then if even after 500 millisec timestamp it shows plural count so return false

/*
    if (current_timestamp >= prev_multiple_count_timestamp + 500ms) {
        if (count > 1) {
            return false;
        }
    }

    //REAL CODE:
    if (!AP_HAL::millis() >= get_multiple_count_start_timestamp() + 500) {
        if (readbuf.count > 1) {
            return false;
        }
    }


    ALSO CODE WRITTEN IN PIXY_PARSER.CPP : RECV_BYTE_PIXY()
*/
    // use data from first (largest) object
    ret.x = -_target_info[0].pos_y;
    ret.y = _target_info[0].pos_x;
    ret.z = 1.0f;
    ret /= ret.length();
    return true;
}
