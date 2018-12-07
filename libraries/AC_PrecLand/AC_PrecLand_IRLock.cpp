
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLock::AC_PrecLand_IRLock(AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock(),
      _have_los_meas(false),
      _los_meas_time_ms(0),
//      _multiple_target_start_flag(false),
//      _multiple_target_timestamp_start(0),
//      _multiple_target_timestamp_latest(0),
      _multiple_target_timestamp_log(),
      _index(0)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock::init() 
{
    irlock.init(get_bus());
}


void AC_PrecLand_IRLock::multiple_target_check() 
{
    if (_index == 24) {
        _index = 0;
    }
    _multiple_target_timestamp_log[_index] = irlock.num_targets();
    _index++;

    printf("\n _multiple_target_timestamp_log: ");
    for (size_t j=0; j<24; j++) {
        printf("%u ", _multiple_target_timestamp_log[j]);
    }

    size_t count = 0;
    for (size_t j=0; j<24; j++) {
        if (_multiple_target_timestamp_log[j] > 1) {
            count++;
        }
    }
    uint16_t multiple_target_count_percentage = count*100/25; 
    printf("\nCount: %u  --  Multiple_target_count_percentage: %u", irlock.num_targets(), multiple_target_count_percentage);     
    if (multiple_target_count_percentage >= 80) {
        printf("\n----------------------------------------------------------------------HIGH COUNT PERCENTAGE: %u  ------setting TARGET ACQUIRED FALSE -- %", multiple_target_count_percentage); 
//            count = 0;
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
    
//    size_t no_targets_tmp = irlock.num_targets();
//    printf("\nOUTSIDE  Number of targets - BEFORE %u", irlock.num_targets());
//    printf("\nHEALTHY: %u", irlock.healthy());  

    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms) {

//        printf("\nNumber of targets OR Count: %u", irlock.num_targets());
        printf("\n\nAP_HAL::millis(): %u   -   _los_meas_time_ms: %u  -  irlock.last_update_ms(): %u",AP_HAL::millis(), _los_meas_time_ms, irlock.last_update_ms());

        irlock.get_unit_vector_body(_los_meas_body);
        _have_los_meas = true;
        _los_meas_time_ms = irlock.last_update_ms();

//        printf("\n_multiple_target_start_flag: %u", _multiple_target_start_flag);
        multiple_target_check(); 

        
/*

        // Logic for Contingency Landing
        if (irlock.num_targets() > 1) {  // If found multiple targets

            if (_multiple_target_start_flag == 0) {     //If it is the 1st time found multiple targets so put the value inside a "start variale"
                _multiple_target_timestamp_start = AP_HAL::millis();
                _multiple_target_start_flag = 1;
            }

            if (_multiple_target_start_flag == 1) {     //If multiple target is seen again i.e. repeated so put the value inside "other variable"
                _multiple_target_timestamp_latest = AP_HAL::millis();
            }

            if (_multiple_target_timestamp_latest - _multiple_target_timestamp_start > 500) {   //If we are getting multiple targets for more than 500ms, go to contigency
//                _frontend.target_acquired(true);   
                printf("\n----------setting TARGET ACQUIRED FALSE"); 
                _frontend.set_target_acquired(false);
//                _frontend._target_acquired = false;
//                _frontend._estimator_initialized = false;
            }
        }
        else {  // If found a single value  within 500 ms so put the start_flag = 0 and restart looking for the multiple target
             _multiple_target_timestamp_latest = AP_HAL::millis();
            if (_multiple_target_timestamp_latest - _multiple_target_timestamp_start < 500) {   //If we are getting multiple targets for more than 500ms, go to contigency
                _multiple_target_timestamp_start = 0;
                _multiple_target_start_flag = 0;
            }
        }
*/
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
