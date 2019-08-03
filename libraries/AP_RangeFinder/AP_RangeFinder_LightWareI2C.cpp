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
#include <assert.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 5

const size_t lx20_max_reply_len_bytes = 32;
const size_t lx20_max_expected_stream_reply_len_bytes = 14;

#define stream_the_median_distance_to_the_first_return "ldf,0"
#define stream_the_raw_distance_to_the_first_return    "ldf,1"
#define stream_the_signal_strength_first_return        "lhf"
#define stream_the_raw_distance_to_the_last_return     "ldl,1"
#define stream_the_signal_strength_last_return         "lhl"
#define stream_the_level_of_background_noise           "ln"

#if SF20_TEST_CODE
/* Data streams from the LiDAR can include any sf20 LiDAR measurement.
 * A request to stream the desired measurement is made on a 20Hz basis and
 * on the next 20Hz service 50ms later, the result is read and a streaming
 * request is made for the next desired measurement in the sequence.
 * Results are generally available from the LiDAR within 10mS of request.
 */
#define STREAM1_VAL stream_the_raw_distance_to_the_first_return
#define STREAM2_VAL stream_the_signal_strength_first_return
#define STREAM3_VAL stream_the_raw_distance_to_the_last_return
#define STREAM4_VAL stream_the_signal_strength_last_return
#define STREAM5_VAL stream_the_level_of_background_noise
const char *parse_stream_id[NUM_TEST_STREAMS] = {
            STREAM1_VAL ":",
            STREAM2_VAL ":",
            STREAM3_VAL ":",
            STREAM4_VAL ":",
            STREAM5_VAL ":"
};

const char *init_stream_id[NUM_TEST_STREAMS] = {
        "$1" STREAM1_VAL "\r\n",
        "$1" STREAM2_VAL "\r\n",
        "$1" STREAM3_VAL "\r\n",
        "$1" STREAM4_VAL "\r\n",
        "$1" STREAM5_VAL "\r\n"
};

const int streamSequence[] = { 0,1,2,3,4 }; // List of 0 based stream Ids that determine the LiDAR values collected.
#endif // SF20_TEST_CODE
const int numStreamSequenceIndexes = sizeof(streamSequence)/sizeof(streamSequence[0]);

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
    if (!dev) {
        return nullptr;
    }

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
        assert(!(expected_reply_len > lx20_max_reply_len_bytes) ||
        (expected_reply_len < 2));
        hal.console->printf("LiDAR_SF20 [%s] len FAILED: %s\n", send_msg, (char*)expected_reply); // TODO: Change to assert
        return false;
    }

    rx_bytes[expected_reply_len] = 0;
    rx_bytes[2] = 0;

    if (!write_bytes((uint8_t*)send_msg,
                           strlen(send_msg))) {
        hal.console->printf("LiDAR_SF20 [%s] 0 FAILED.\n", (char*)send_msg); // TODO: Remove except in test branch.
        return false;
    }

    if (!sf20_wait_on_reply(rx_bytes)) {
        hal.console->printf("LiDAR_SF20 [%s] 1 FAILED: %s\n", send_msg, (char*)rx_bytes); // TODO: Remove except in test branch.
        return false;
    }

    if ((rx_bytes[0] != expected_reply[0]) ||
        (rx_bytes[1] != expected_reply[1]) ) {
        hal.console->printf("LiDAR_SF20 [%s] 2 FAILED: %s\n", send_msg, (char*)rx_bytes); // TODO: Remove except in test branch.
        return false;
    }

    if (!_dev->read(rx_bytes, expected_reply_len)) {
        hal.console->printf("LiDAR_SF20 [%s] 3 FAILED: %s\n", send_msg, (char*)rx_bytes); // TODO: Remove except in test branch.
        return false;
    }

    for (int i = 0 ; i < expected_reply_len ; i++) {
        if (rx_bytes[i] != expected_reply[i]) {
            hal.console->printf("LiDAR_SF20 [%s] 4 FAILED: %s\n", send_msg, (char*)rx_bytes); // TODO: Remove except in test branch.
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
    missed_samples = 0;

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

#if 0 // Address change to 0x65 = 101
    write_bytes((uint8_t*)"#CI,0x65\r\n",10);
    _dev->set_address(0x65);
    uint8_t rx_bytes[lx20_max_reply_len_bytes + 1];
    sf20_wait_on_reply(rx_bytes);
    // Save the comm settings
    if(!sf20_send_and_expect("%C\r\n", "%c:")) {
        return false;
    }
#endif

    /* Sets the Laser Encoding to a fixed pattern to assess how well it improves operation with interference from
     * the Precision Landing infrared beacon.
     */
	// Changes the laser encoding pattern: 3 (Random A) [0..4].
    if(!sf20_send_and_expect("#LE,3\r\n", "le:3")) {
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


    // Requests the measurement specified in the first stream.
    int i = 0;
    write_bytes((uint8_t*)init_stream_id[i], strlen(init_stream_id[i]));

#if 0 // If testing indicates that the legacy mode is appropriate, legacy binary streaming may be used.
    // Enable I2C binary distance streaming.
    // This enables the output of binary coded distance in centimeters.
    // Sending any other command will disable it.
    const uint8_t send_buf_enable_I2C_legacy_distance_streaming[] = "0'";
    if (!write_bytes((uint8_t*)send_buf_enable_I2C_legacy_distance_streaming,
                        strlen(send_buf_enable_I2C_legacy_distance_streaming))) {
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
    // Parses up to 5 ASCII streams for LiDAR data.
    // If a parse fails, the stream measurement is not updated until it is successfully read in the future.
    uint8_t stream[lx20_max_expected_stream_reply_len_bytes]; // Maximum response length for a stream ie "ldf,0:40.99" is 11 characters

    bool ret;
    int i;

    /* Reads the LiDAR value requested during the last interrupt. */
    ret = _dev->read(stream, sizeof(stream));
    if (ret) {
        i = streamSequence[currentStreamSequenceIndex];
        size_t num_processed_chars = 0;
        if (sf20_parse_stream(stream, &num_processed_chars, parse_stream_id[i], sf20_test_val[i])) {
            switch (i) {
            case 0:
                reading_cm = sf20_test_val[0];
                break;
            case 1:
                break;
            }
        }
        else {
            // Count issues
            missed_samples++;
            state.voltage_mv = missed_samples;
        }
    }

    currentStreamSequenceIndex++;
    if (currentStreamSequenceIndex >= numStreamSequenceIndexes) {
        currentStreamSequenceIndex = 0;
        // Logs collected data:
        data_log(sf20_test_val);
    }
    i = streamSequence[currentStreamSequenceIndex];

    write_bytes((uint8_t*)init_stream_id[i], strlen(init_stream_id[i]));

    return ret;
}

void AP_RangeFinder_LightWareI2C::data_log(uint16_t *val)
{
    DataFlash_Class::instance()->Log_Write("SF20", "Time_uS,s0,s1,s2,s3,s4",
                                                              "QHHHHH",
                                            AP_HAL::micros64(),
                                            val[0], val[1], val[2], val[3], val[4]);
}


bool AP_RangeFinder_LightWareI2C::sf20_parse_stream(uint8_t *stream_buf,
                       size_t *p_num_processed_chars,
                       const char *string_identifier,
                       uint16_t &val) {
    size_t string_identifier_len = strlen(string_identifier);
    for (int i = 0 ; i < string_identifier_len ; i++) {
        if (stream_buf[*p_num_processed_chars] != string_identifier[i]) {
            return false;
        }
        (*p_num_processed_chars)++;
    }

    /* Number is always returned in hundredths. So 6.33 is returned as 633. 6.3 is returned as 630.
         * 6 is returned as 600.
         * Extract number in format 6.33 or 123.99 (meters to be converted to centimeters).
         * Percentages such as 100 (percent), are returned as 10000.
         */
    uint32_t final_multiplier = 100;
    bool decrement_multiplier = false;
    bool number_found = false;
    uint16_t accumulator = 0;
    uint16_t digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
    while ((((digit_u16 < '9') &&
             (digit_u16 >= '0')) ||
             (digit_u16 == '.')) &&
             (*p_num_processed_chars < lx20_max_reply_len_bytes)) {
        (*p_num_processed_chars)++;
        if (decrement_multiplier) {
            final_multiplier /=10;
        }
        if (digit_u16 == '.') {
            decrement_multiplier = true;
            digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
            continue;
        }
        number_found = true;
        accumulator *= 10;
        accumulator += digit_u16 - '0';
        digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
    }

    accumulator *= final_multiplier;
    val = accumulator;
    return number_found;
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
            const char fmt_wait_ms[] = "SF20 wait: normal exit after %d ms"; //TODO: remove after testing all printf
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
