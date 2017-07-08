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
//#include "AP_RangeFinder_LightWareI2C_lw20api.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 5

//lwLW20			lw20;
const uint32_t _headerSize_bytes_ = 0; // Must match definition in send_buf_disable_fx20_address_tagging.
                                       // The address field ("0x66")is optionally included in the byte_rx buffer and may be accounted for here as a 4 byte header.
const uint32_t _lx20_max_reply_len_bytes_ = 32;
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

/**
 * Wrapper function over #transfer() to write a sequence of bytes to
 * device. No values are read.
 */
bool AP_RangeFinder_LightWareI2C::write_bytes(uint8_t *write_buf_u8, uint32_t len_u8)
{
    return _dev->transfer(write_buf_u8, len_u8, NULL, 0);
}

/**
 * Disables "address tagging" in the fx20 response packets.
 */
bool AP_RangeFinder_LightWareI2C::fx20_disable_address_tagging()
{
    if(!fx20_send_and_expect("#CT,0\r\n", "ct:0")) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_LightWareI2C::fx20_product_name_check()
{
    if(!fx20_send_and_expect("?P\r\n", "p:LW21,")) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_LightWareI2C::fx20_send_and_expect(const char* send_msg, const char* expected_reply)
{
    uint8_t rx_bytes[_lx20_max_reply_len_bytes_ + 1];
    int expected_reply_len = strlen(expected_reply);

    if ((expected_reply_len > _lx20_max_reply_len_bytes_) ||
        (expected_reply_len < 2)) {
        hal.console->printf("Lidar_FX20 [%s] len FAILED: %s\n", send_msg, (char*)expected_reply);
        return false;
    }

    rx_bytes[expected_reply_len] = 0;
    rx_bytes[2] = 0;

    if (!write_bytes((uint8_t*)send_msg,
                           strlen(send_msg) + 1)) {
        hal.console->printf("Lidar_FX20 [%s] 0 FAILED.\n", (char*)send_msg);
        return false;
    }

    if (!fx20_wait_on_reply(rx_bytes)) {
        hal.console->printf("Lidar_FX20 [%s] 1 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    if ((rx_bytes[0] != expected_reply[0]) ||
        (rx_bytes[1] != expected_reply[1]) ) {
        hal.console->printf("Lidar_FX20 [%s] 2 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    if (!_dev->read(rx_bytes, expected_reply_len)) {
        hal.console->printf("Lidar_FX20 [%s] 3 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    for (int i = 0 ; i < expected_reply_len ; i++) {
        if (rx_bytes[i] != expected_reply[i]) {
            hal.console->printf("Lidar_FX20 [%s] 4 FAILED: %s\n", send_msg, (char*)rx_bytes);
            return false;
        }
    }

    return true;
}

/* Driver first attempts to initialize the fx20.
 * If for any reason this fails, the driver attempts to initialize the legacy LightWare lidar.
 * If this fails, the driver returns false indicating no LightWare lidar is present.
 */
bool AP_RangeFinder_LightWareI2C::init()
{
    if (!fx20_init()) {
    	if (!legacy_init()) {
    		return false;
    	}
    }
    return true;
}

// Original Lidar sensor configuration.
bool AP_RangeFinder_LightWareI2C::legacy_init()
{
    union {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retrieve lost signal timeout register
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
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::legacy_timer, void));

    return true;
}

bool AP_RangeFinder_LightWareI2C::fx20_init()
{
    uint8_t byte_rx[33];
    byte_rx[32] = 0;

    // Makes sure that "address tagging" is turned off.
    // Address tagging starts every response with "0x66".
    // Turns off Address Tagging just in case it was previously left on in the non-volatile configuration.
    if(!fx20_disable_address_tagging()) {
    	return false;
    }

    if(!fx20_product_name_check()) {
        return false;
    }

    // Changes the number of lost signal confirmations: 1 [1..250].
    if(!fx20_send_and_expect("#LC,1\r\n", "lc:1")) {
        return false;
    }

    // Assuming there are some external communications prior to this point that will
    // randomize the startup time, the least significant bits of the system clock
    // are used to select the encoding pattern.
	// Changes the laser encoding pattern: 2 [0..4].
#if 0
    if(!fx20_send_and_expect("#LE,2\r\n", "le:2")) {
        return false;
    }

	uint8_t send_buf_change_laser_encoding_pattern[] = "#LE,2\r\n";
	// Pull parameter for encoding and overwrite hardcoded specifier for pattern 2.
#endif
#if 0
    for ( int i = 0 ; i<4 ; i++) {
		// Changes the laser encoding pattern: 2 [0..4].
		uint8_t send_buf_change_laser_encoding_pattern[] = "#LE,2\r\n";
		// Pull parameter for encoding and overwrite hardcoded specifier for pattern 2.
//    hal.console->printf(fmt_laser_encoding_pattern, (char*)byte_rx + _headerSize_bytes_);
//    Copter::gcs_send_text(MAV_SEVERITY_CRITICAL, byte_rx);
#endif


#if 0
    // Enable I2C legacy distance streaming
    const uint8_t send_buf_enable_I2C_legacy_distance_streaming[] = "0'";
    if (!_dev->transfer((uint8_t*)send_buf_enable_I2C_legacy_distance_streaming,
                           sizeof(send_buf_enable_I2C_legacy_distance_streaming),
						   byte_rx,
						   0)) {
        return false;
    }
#else
//    const char stream_the_median_distance_to_the_first_return_on_stream_one[] = "$1,ldf\r";
    const char stream_the_raw_distance_to_the_first_return_on_stream_one[] = "$1,ldf,1\r";
    if (!_dev->transfer((uint8_t*)stream_the_raw_distance_to_the_first_return_on_stream_one,
                           sizeof(stream_the_raw_distance_to_the_first_return_on_stream_one),
						   byte_rx,
						   10)) {
        return false;
    }

    // TODO: Currently ignores response.

    // Enable I2C binary distance streaming.
    // This enables output binary coded distance in centimeters.
    // Sending any other command will disable it.
    const uint8_t send_buf_enable_I2C_legacy_distance_streaming[] = "0'";
    if (!_dev->transfer((uint8_t*)send_buf_enable_I2C_legacy_distance_streaming,
                           sizeof(send_buf_enable_I2C_legacy_distance_streaming),
						   byte_rx,
						   0)) {
        return false;
    }
#endif

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::fx20_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::legacy_get_reading(uint16_t &reading_cm)
{
#if 0 // latest from git hub master.
    be16_t val;

//    if (ranger._address[state.instance] == 0) {
//        return false;
//    }

    // read the high and low byte distance registers
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);
    }

return ret;
#else //lateset from git hub mttr/master
    be16_t val;

    const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
        // combine results into distance
        reading_cm = be16toh(val);
        return true;
    }
    return false;
#endif //lateset from git hub mttr/master
}

// read - return last value measured by fx20 sensor
bool AP_RangeFinder_LightWareI2C::fx20_get_reading(uint16_t &reading_cm)
{
    be16_t val;

    // Reads the streams in 16 bit binary.
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);
    }

return ret;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareI2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_LightWareI2C::legacy_timer(void)
{
    if (legacy_get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

void AP_RangeFinder_LightWareI2C::fx20_timer(void)
{
    if (fx20_get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// Only for use during init as this blocks while waiting for the FX20 to be ready.
bool AP_RangeFinder_LightWareI2C::fx20_wait_on_reply(uint8_t *rx_two_byte)
{
    // Waits for a non-zero first byte while repeatedly reading 16 bits.
    // This is used after a read command to allow the fx20 time to provide the result.
    uint32_t start_time_ms = AP_HAL::millis();
    uint32_t current_time_ms;
    uint32_t elapsed_time_ms;
    const uint32_t max_wait_time_ms = 50;

    while(_dev->read(rx_two_byte, 2)) {
        current_time_ms = AP_HAL::millis();
        elapsed_time_ms = current_time_ms - start_time_ms;
        if (rx_two_byte[0] != 0) {
            const char fmt_wait_ms[] = "FX20 wait: normal exit after %d ms";
            hal.console->printf(fmt_wait_ms, elapsed_time_ms);
            return true;
        }
        if (elapsed_time_ms > max_wait_time_ms) {
            const char fmt_wait_timeout_ms[] = "FX20 wait: timeout exit after %d ms";
            hal.console->printf(fmt_wait_timeout_ms, elapsed_time_ms);
            return false;
        }
    }
    current_time_ms = AP_HAL::millis();
    elapsed_time_ms = current_time_ms - start_time_ms;
    const char fmt_wait_fail_ms[] = "FX20 wait: fail exit after %d ms";
    hal.console->printf(fmt_wait_fail_ms, elapsed_time_ms);
    return false;
}
