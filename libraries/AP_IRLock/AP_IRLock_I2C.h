/*
 * AP_IRLock_I2C.h
 *
 */
#pragma once

#include "IRLock.h"
#include <AP_HAL/AP_HAL.h>

class AP_IRLock_I2C : public IRLock
{
public:
    // init - initialize sensor library
    void init(int8_t bus) override;

    // returns the number of blocks in the current frame
    size_t num_targets() const override { return _block_cnt; }

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    struct PACKED frame {
        uint16_t checksum;
        uint16_t signature;
        uint16_t pixel_x;
        uint16_t pixel_y;
        uint16_t pixel_size_x;
        uint16_t pixel_size_y;
    };

    bool timer(void);

    bool sync_frame_start(void);
    bool read_block(struct frame &irframe);
    size_t read_blocks(struct frame *blocks, size_t max_cnt);
    void read_frames(void);

    void pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y);

    HAL_Semaphore sem;
    uint32_t _last_read_ms;
    size_t _block_cnt;
};
