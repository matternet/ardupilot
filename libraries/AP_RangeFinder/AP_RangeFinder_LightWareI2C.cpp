/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RangeFinder_LightWareI2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 5

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_LightWareI2C::detect(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_RangeFinder_LightWareI2C *sensor
        = new AP_RangeFinder_LightWareI2C(_state, std::move(dev));

    if (!sensor) {
        goto fail;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!sensor->init()) {
            sensor->_dev->get_semaphore()->give();
            goto fail;
        }

        sensor->_dev->get_semaphore()->give();
        return sensor;
    } else {
        goto fail;
    }

fail:
    delete sensor;
    return nullptr;
}

bool AP_RangeFinder_LightWareI2C::init()
{
    union {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retreive lost signal timeout register
    const uint8_t read_reg = LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG;
    if (!_dev->transfer(&read_reg, 1, timeout.bytes, 2)) {
        return false;
    }

    // Check lost signal timeout register against desired value and write it if it does not match
    if (be16toh(timeout.be16_val) != LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE) {
        timeout.be16_val = htobe16(LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE);
        const uint8_t send_buf[3] = {LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG, timeout.bytes[0], timeout.bytes[1]};
        if (!_dev->transfer(send_buf, sizeof(send_buf), nullptr, 0)) {
            return false;
        }
    }

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::get_reading(uint16_t &reading_cm)
{
    be16_t val;

    if (state.address == 0) {
        return false;
    }

    const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
        // combine results into distance
        reading_cm = be16toh(val);
        return true;
    }
    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareI2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_LightWareI2C::timer(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
