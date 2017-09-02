#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AC_PrecLand_IRLockPozyx.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLockPozyx::AC_PrecLand_IRLockPozyx(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
: AC_PrecLand_IRLock(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLockPozyx::init()
{
    AC_PrecLand_IRLock::init();
    _pozyx_dev = hal.i2c_mgr->get_device(0, 0x4B);
    _pozyx_dev->set_split_transfers(false);

    if (!_pozyx_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    // check whoami
    {
        uint8_t whoami_reg = 0x00; // WHOAMI
        uint8_t whoami = 0;

        if (!_pozyx_dev->transfer(&whoami_reg, 1, &whoami, 1) || whoami != 0x43) {
            _pozyx_dev->get_semaphore()->give();
            return;
        }
    }

    _pozyx_dev->get_semaphore()->give();

    // 20hz
    _pozyx_dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&AC_PrecLand_IRLockPozyx::timer, void));
}

float AC_PrecLand_IRLockPozyx::distance_to_target() {

    if (_have_range_meas) {
        uint8_t idx = _range_data_idx;
        return MAX(_range_data[idx].range_m, 0.01f);
    }
    return 0.0f;
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_IRLockPozyx::update()
{
    AC_PrecLand_IRLock::update();
    _have_range_meas = _have_range_meas && (AP_HAL::micros()-_range_data[_range_data_idx].time_us < 150000);

}

void AC_PrecLand_IRLockPozyx::timer()
{
    uint8_t interrupts = 0;

    {
        uint8_t interrupts_reg = 0x05; // WHOAMI
        _pozyx_dev->transfer(&interrupts_reg, 1, &interrupts, 1);
    }

    if (interrupts & (1<<4)) { // FUNC interrupt
        struct PACKED {
            uint8_t reg = 0xC7; // GETRANGEINFO
            le16_t remote_id;
        } req;

        req.remote_id = htole16(_remote_netid);

        struct PACKED {
            uint8_t success;
            le32_t pozyx_time_ms;
            le32_t range;
            le16_t RSS;
        } res;

        _pozyx_dev->transfer((uint8_t*)&req, sizeof(req), (uint8_t*)&res, sizeof(res));

        uint8_t next_range_data_idx = (_range_data_idx+1)%2;
        _range_data[next_range_data_idx].range_m = le32toh(res.range)*1e-3f;
        _range_data[next_range_data_idx].time_us = AP_HAL::micros();
        _range_data_idx = next_range_data_idx;
        _have_range_meas = true;
    }

    {
        struct PACKED {
            uint8_t reg = 0xB5; // DO_RANGING
            le16_t remote_id;
        } req;
        req.remote_id = htole16(_remote_netid);

        struct PACKED {
            uint8_t success;
        } res;

        _pozyx_dev->transfer((uint8_t*)&req, sizeof(req), (uint8_t*)&res, sizeof(res));
    }
}
