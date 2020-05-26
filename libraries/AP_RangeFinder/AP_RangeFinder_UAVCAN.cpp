#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_RangeFinder_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if HAL_OS_POSIX_IO
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

extern const AP_HAL::HAL& hal;

#define debug_rangefinder_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/*
  constructor - registers instance at top Rangefinder driver
 */
AP_RangeFinder_UAVCAN::AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_state)
{
    _sem = hal.util->new_semaphore();

    _last_init_check_ms = AP_HAL::millis();
    init_rangefinder();
}

/*
  try to associate this backend instance with a uavcan rangefinder
  node
 */
void AP_RangeFinder_UAVCAN::init_rangefinder()
{
    if (_initialized) {
        return;
    }
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        uint8_t free = ap_uavcan->find_smallest_free_rangefinder_node();
        if (free == UINT8_MAX) {
            continue;
        }
        if (register_uavcan_rangefinder(i, free)) {
            break;
        }
    }
}

AP_RangeFinder_UAVCAN::~AP_RangeFinder_UAVCAN()
{
    if (!_initialized) {
        return;
    }
    
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(_manager);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    ap_uavcan->remove_rangefinder_listener(this);
    delete _sem;
    _sem = nullptr;
    
    debug_rangefinder_uavcan(2, "AP_RangeFinder_UAVCAN destructed\n\r");
}

// Read the sensor
void AP_RangeFinder_UAVCAN::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (!_initialized && now - _last_init_check_ms >= 1000) {
        _last_init_check_ms = now;
        init_rangefinder();
    }
    _sem->take_blocking();
    if (now - _last_update_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    } else {
        state.distance_cm = _distance * 100;
        set_status(_status);
    }
    _sem->give();
}

void AP_RangeFinder_UAVCAN::handle_rangefinder_msg(float distance, RangeFinder::RangeFinder_Status status)
{
    _sem->take_blocking();
    _distance = distance;
    _status = status;
    _last_update_ms = AP_HAL::millis();
    _sem->give();
}

bool AP_RangeFinder_UAVCAN::register_uavcan_rangefinder(uint8_t mgr, uint8_t node)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return false;
    }
    _manager = mgr;

    if (ap_uavcan->register_rangefinder_listener_to_node(this, node)) {
        debug_rangefinder_uavcan(2, "AP_RangeFinder_UAVCAN loaded\n\r");

        _initialized = true;

        return true;
    }

    return false;
}

#endif // HAL_WITH_UAVCAN

