/*
 * AP_IRLock_I2C.h
 *
 */
#pragma once

#include "IRLock.h"
#include "pixy_parser.h"

class AP_IRLock_I2C : public IRLock
{
public:
    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

    pixy_parser pixyObj;

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    void read_frames(void);
    void pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y);
//    void convert_pixy_data(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

    AP_HAL::Semaphore *sem;
    uint32_t _last_read_ms;
    size_t target_count;
};