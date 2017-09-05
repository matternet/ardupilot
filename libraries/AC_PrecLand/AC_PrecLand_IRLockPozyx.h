#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_IRLock/AP_IRLock.h>
#include "AC_PrecLand_IRLock.h"
#include <AP_HAL/I2CDevice.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_IRLock/AP_IRLock_SITL.h>
#endif

/*
 * AC_PrecLand_IRLockPozyx - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_IRLockPozyx : public AC_PrecLand_IRLock
{
public:

    // Constructor
    AC_PrecLand_IRLockPozyx(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    float distance_to_target() override;

private:
    void timer();

    struct {
        float range_m;
        uint32_t time_us;
    } _range_data[2];
    volatile uint8_t _range_data_idx;
    volatile bool _have_range_meas;
    uint32_t _prev_range_time_us;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _pozyx_dev;

    uint16_t _remote_netid = 0x6154;
};
