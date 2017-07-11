#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_LightWareI2C : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

private:
    // constructor
    AP_RangeFinder_LightWareI2C(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    bool sf20_disable_address_tagging();
    bool sf20_product_name_check();
    bool sf20_send_and_expect(const char* send, const char* expected_reply);
    bool sf20_set_lost_signal_confirmations();
    bool sf20_wait_on_reply(uint8_t *rx_two_bytes);
    bool init();
    bool legacy_init();
    bool sf20_init();
    void legacy_timer();
    void sf20_timer();

    // get a reading
    bool legacy_get_reading(uint16_t &reading_cm);
    bool sf20_get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
