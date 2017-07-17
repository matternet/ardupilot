#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define SF20_TEST_CODE 1
#if SF20_TEST_CODE
    #define NUM_TEST_STREAMS 5
#endif

class AP_RangeFinder_LightWareI2C : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    int missed_samples;

#if SF20_TEST_CODE
    uint16_t sf20_test_val[NUM_TEST_STREAMS];
    int currentStreamSequenceIndex = 0;
#endif

    // constructor
    AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    bool sf20_disable_address_tagging();
    bool sf20_product_name_check();
    bool sf20_send_and_expect(const char* send, const char* expected_reply);
    bool sf20_set_lost_signal_confirmations();
    bool sf20_wait_on_reply(uint8_t *rx_two_bytes);
    bool init();
    bool legacy_init();
    bool sf20_init();
    void sf20_init_streamRecovery();
    void legacy_timer();
    void sf20_timer();

    // get a reading
    bool legacy_get_reading(uint16_t &reading_cm);
    bool sf20_get_reading(uint16_t &reading_cm);
    bool sf20_parse_stream(uint8_t *stream_buf,
                           size_t *p_num_processed_chars,
                           const char *string_identifier,
                           uint16_t &val);
    void data_log(uint16_t *val);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
