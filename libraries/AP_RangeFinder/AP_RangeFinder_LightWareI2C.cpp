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
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 5

const size_t lx20_max_reply_len_bytes = 32;
/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   Detects if a Lightware rangefinder is connected. We'll detect by
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
 * Disables "address tagging" in the sf20 response packets.
 */
bool AP_RangeFinder_LightWareI2C::sf20_disable_address_tagging()
{
    if(!sf20_send_and_expect("#CT,0\r\n", "ct:0")) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_LightWareI2C::sf20_product_name_check()
{
    if(!sf20_send_and_expect("?P\r\n", "p:LW20,")) {
        return false;
    }
    return true;
}

bool AP_RangeFinder_LightWareI2C::sf20_send_and_expect(const char* send_msg, const char* expected_reply)
{
    uint8_t rx_bytes[lx20_max_reply_len_bytes + 1];
    size_t expected_reply_len = strlen(expected_reply);

    if ((expected_reply_len > lx20_max_reply_len_bytes) ||
        (expected_reply_len < 2)) {
        hal.console->printf("LiDAR_SF20 [%s] len FAILED: %s\n", send_msg, (char*)expected_reply);
        return false;
    }

    rx_bytes[expected_reply_len] = 0;
    rx_bytes[2] = 0;

    if (!write_bytes((uint8_t*)send_msg,
                           strlen(send_msg) + 1)) {
        hal.console->printf("LiDAR_SF20 [%s] 0 FAILED.\n", (char*)send_msg);
        return false;
    }

    if (!sf20_wait_on_reply(rx_bytes)) {
        hal.console->printf("LiDAR_SF20 [%s] 1 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    if ((rx_bytes[0] != expected_reply[0]) ||
        (rx_bytes[1] != expected_reply[1]) ) {
        hal.console->printf("LiDAR_SF20 [%s] 2 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    if (!_dev->read(rx_bytes, expected_reply_len)) {
        hal.console->printf("LiDAR_SF20 [%s] 3 FAILED: %s\n", send_msg, (char*)rx_bytes);
        return false;
    }

    for (int i = 0 ; i < expected_reply_len ; i++) {
        if (rx_bytes[i] != expected_reply[i]) {
            hal.console->printf("LiDAR_SF20 [%s] 4 FAILED: %s\n", send_msg, (char*)rx_bytes);
            return false;
        }
    }

    return true;
}

/* Driver first attempts to initialize the sf20.
 * If for any reason this fails, the driver attempts to initialize the legacy LightWare LiDAR.
 * If this fails, the driver returns false indicating no LightWare LiDAR is present.
 */
bool AP_RangeFinder_LightWareI2C::init()
{
    if (!sf20_init()) {
    	if (!legacy_init()) {
    		return false;
    	}
    }
    return true;
}

// Original LiDAR sensor configuration.
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

bool AP_RangeFinder_LightWareI2C::sf20_init()
{
    // Makes sure that "address tagging" is turned off.
    // Address tagging starts every response with "0x66".
    // Turns off Address Tagging just in case it was previously left on in the non-volatile configuration.
    if(!sf20_disable_address_tagging()) {
    	return false;
    }

    if(!sf20_product_name_check()) {
        return false;
    }

    // Disconnect the servo.
    if(!sf20_send_and_expect("#SC,0\r\n", "sc:0")) {
        return false;
    }

    // Change the power consumption:
    // 0 = power off
    // 1 = power on
    // As of 7/10/17 sw and fw version 1.0 the "#E,1" command does not seem to be supported.
    // When it is supported the expected response would be "e:1".

    // Changes the number of lost signal confirmations: 1 [1..250].
    if(!sf20_send_and_expect("#LC,1\r\n", "lc:1")) {
        return false;
    }

    // For now just set to a fixed pattern to assess how well it improves operation with beacon.
    // Pull parameter for encoding and overwrite hard-coded specifier for pattern 2.
    // Todo: This is not useful. Have a way to set it on take off or landing.
    // Assuming there are some external communications prior to this point that will
    // randomize the startup time, the least significant bits of the system clock
    // are used to select the encoding pattern.
	// Changes the laser encoding pattern: 2 [0..4].
    if(!sf20_send_and_expect("#LE,2\r\n", "le:2")) {
        return false;
    }

    // Sets datum offset [-10.00 ... 10.00].
    if(!sf20_send_and_expect("#LO,0.00\r\n", "lo:0.00")) {
        return false;
    }

    // Changes to a new measuring mode (update rate):
    //    1 = 388 readings per second
    //    2 = 194 readings per second
    //    3 = 129 readings per second
    //    4 = 97 readings per second
    //    5 = 78 readings per second
    //    6 = 65 readings per second
    //    7 = 55 readings per second
    //    8 = 48 readings per second
    if(!sf20_send_and_expect("#LM,7\r\n", "lm:7")) {
        return false;
    }

    // Changes the gain boost value:
    //     Adjustment range = -20.00 ... 5.00
    if(!sf20_send_and_expect("#LB,0.00\r\n", "lb:0.00")) {
        return false;
    }

    // Switches distance streaming on or off:
    // 0 = off
    // 1 = on
    if(!sf20_send_and_expect("#SU,1\r\n", "su:1")) {
        return false;
    }

    // Changes the laser state:
    //    0 = laser is off
    //    1 = laser is running
    if(!sf20_send_and_expect("#LF,1\r\n", "lf:1")) {
        return false;
    }

    if(!sf20_send_and_expect("?LT,1\r\n", "lt:")) {
        return false;
    }

#if 1
    be16_t reading_cm;
    sf20_get_reading(reading_cm);
#endif

    // Configures the first stream for the raw first return.
    // Alternatively for the median use:
    // const char stream_the_median_distance_to_the_first_return_on_stream_one[] = "$1,ldf\r";
    const char stream_the_raw_distance_to_the_first_return_on_stream_one[] = "$1ldf,1\r\n";
    if (!write_bytes((uint8_t*)stream_the_raw_distance_to_the_first_return_on_stream_one,
                        sizeof(stream_the_raw_distance_to_the_first_return_on_stream_one))) {
        return false;
    }

#if 0
    // Signal strength is returned as a (%)
    const char stream_the_signal_strength_first_return_on_stream_2[] = "$2lhf\n";
    if (!write_bytes((uint8_t*)stream_the_signal_strength_first_return_on_stream_2,
                        sizeof(stream_the_signal_strength_first_return_on_stream_2))) {
        return false;
    }
#endif

#if 0
    // Streams the raw distance to the last return on stream two.
    const char stream_the_raw_distance_to_the_last_return_on_stream_two[] = "$3ldf,1\r\n"; //$3ldl,1\r\n
    if (!write_bytes((uint8_t *)stream_the_raw_distance_to_the_last_return_on_stream_two,
                         sizeof(stream_the_raw_distance_to_the_last_return_on_stream_two))) {
        return false;
    }
#endif

#if 0
        // Signal strength is returned as a (%)
    const char stream_the_signal_strength_last_return_on_stream_4[] = "$4lhl\r\n";
    if (!write_bytes((uint8_t*)stream_the_signal_strength_last_return_on_stream_4,
                        sizeof(stream_the_signal_strength_last_return_on_stream_4))) {
        return false;
    }

    // Streams the level of background noise.
    const char stream_the_level_of_background_noise_on_stream_5[] = "$5ln\r\n";
    if (!write_bytes((uint8_t*)stream_the_level_of_background_noise_on_stream_5,
                        sizeof(stream_the_level_of_background_noise_on_stream_5))) {
    return false;
}
#endif

#if 1
    // Enable I2C binary distance streaming.
    // This enables the output of binary coded distance in centimeters.
    // Sending any other command will disable it.
    const uint8_t send_buf_enable_I2C_legacy_distance_streaming[] = "0'";
    if (!write_bytes((uint8_t*)send_buf_enable_I2C_legacy_distance_streaming,
                        sizeof(send_buf_enable_I2C_legacy_distance_streaming))) {
        return false;
    }
#endif
    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::sf20_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::legacy_get_reading(uint16_t &reading_cm)
{
#if 0 // latest from git hub master.
    be16_t val;

    if (ranger._address[state.instance] == 0) {
        return false;
    }

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

// read - return last value measured by sf20 sensor
bool AP_RangeFinder_LightWareI2C::sf20_get_reading(uint16_t &reading_cm)
{
#if 0
    be16_t val[15];
    const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
        // combine results into distance
        reading_cm = be16toh(val[0]);
        return true;
    }
    return false;
#else
    be16_t val[15];

    // Reads the streams in 16 bit binary.
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val[0]);
    }

    return ret;
#endif
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

void AP_RangeFinder_LightWareI2C::sf20_timer(void)
{
    if (sf20_get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// Only for use during init as this blocks while waiting for the SF20 to be ready.
bool AP_RangeFinder_LightWareI2C::sf20_wait_on_reply(uint8_t *rx_two_byte)
{
    // Waits for a non-zero first byte while repeatedly reading 16 bits.
    // This is used after a read command to allow the sf20 time to provide the result.
    uint32_t start_time_ms = AP_HAL::millis();
    uint32_t current_time_ms;
    uint32_t elapsed_time_ms;
    const uint32_t max_wait_time_ms = 50;

    while(_dev->read(rx_two_byte, 2)) {
        current_time_ms = AP_HAL::millis();
        elapsed_time_ms = current_time_ms - start_time_ms;
        if (rx_two_byte[0] != 0) {
            const char fmt_wait_ms[] = "SF20 wait: normal exit after %d ms";
            hal.console->printf(fmt_wait_ms, elapsed_time_ms);
            return true;
        }
        if (elapsed_time_ms > max_wait_time_ms) {
            const char fmt_wait_timeout_ms[] = "SF20 wait: timeout exit after %d ms";
            hal.console->printf(fmt_wait_timeout_ms, elapsed_time_ms);
            return false;
        }
    }
    current_time_ms = AP_HAL::millis();
    elapsed_time_ms = current_time_ms - start_time_ms;
    const char fmt_wait_fail_ms[] = "SF20 wait: fail exit after %d ms";
    hal.console->printf(fmt_wait_fail_ms, elapsed_time_ms);
    return false;
}
