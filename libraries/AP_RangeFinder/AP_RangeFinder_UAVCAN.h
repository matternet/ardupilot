#pragma once

#include "RangeFinder_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_RangeFinder_UAVCAN : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &_state);
    ~AP_RangeFinder_UAVCAN() override;

    void update() override;

    // This method is called from UAVCAN thread
    void handle_rangefinder_msg(float distance, RangeFinder::RangeFinder_Status status) override;

    bool register_uavcan_rangefinder(uint8_t mgr, uint8_t node);

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    uint8_t _instance;
    float _distance;
    uint32_t _last_update_ms;
    RangeFinder::RangeFinder_Status _status;
    uint8_t _manager;
    AP_HAL::Semaphore *_sem;
    uint32_t _last_init_check_ms;

    void init_rangefinder();

    bool _initialized;
};
