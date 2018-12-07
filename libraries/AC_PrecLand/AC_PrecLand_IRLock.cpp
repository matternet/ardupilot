
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLock::AC_PrecLand_IRLock(AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock(),
      _have_los_meas(false),
      _los_meas_time_ms(0),
      _multiple_target_timestamp_log(),
      _index(0)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock::init() 
{
    irlock.init(get_bus());
}

//multiple_target_check - Checks if multiple targets detected for 500 ms continously. If yes, sets target_acquired=False which calls contingency landing
void AC_PrecLand_IRLock::multiple_target_check() 
{
    if (_index == 24) {
        _index = 0;
    }
    _multiple_target_timestamp_log[_index] = irlock.num_targets();
    _index++;

/*
    printf("\n _multiple_target_timestamp_log: ");
    for (size_t j=0; j<24; j++) {
        printf("%u ", _multiple_target_timestamp_log[j]);
    }
*/
    size_t count = 0;
    for (size_t j=0; j<24; j++) {
        if (_multiple_target_timestamp_log[j] > 1) {
            count++;
        }
    }
    uint16_t multiple_target_count_percentage = count*100/25; 
//    printf("\nCount: %u  --  Multiple_target_count_percentage: %u", irlock.num_targets(), multiple_target_count_percentage);     
    if (multiple_target_count_percentage >= 80) {
//        printf("\n----------------------------------------------------------------------HIGH COUNT PERCENTAGE: %u  ------setting TARGET ACQUIRED FALSE -- %", multiple_target_count_percentage); 
        _frontend.set_target_acquired(false); 
    }
}


// update - give chance to driver to get updates from sensor
void AC_PrecLand_IRLock::update()
{
    // update health
    _state.healthy = irlock.healthy();
    
    // get new sensor data
    irlock.update();
    
    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms) {
        irlock.get_unit_vector_body(_los_meas_body);
        _have_los_meas = true;
        _los_meas_time_ms = irlock.last_update_ms();
        multiple_target_check(); 
    }
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_IRLock::get_los_body(Vector3f& ret) {
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_IRLock::los_meas_time_ms() {
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_IRLock::have_los_meas() {
    return _have_los_meas;
}
