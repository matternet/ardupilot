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

/*
 * AP_IRLock_I2C.cpp
 *
 * Based on AP_IRLock_PX4 by MLandes
 *
 * See: http://irlock.com/pages/serial-communication-protocol
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_IRLock_I2C.h"
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

#define IRLOCK_I2C_ADDRESS      0x54



void AP_IRLock_I2C::init(int8_t bus) {
    if (bus < 0) {
        // default to i2c external bus
        bus = 1;
    }
    dev = std::move(hal.i2c_mgr->get_device(bus, IRLOCK_I2C_ADDRESS));
    if (!dev) {
        return;
    }
    sem = hal.util->new_semaphore();

//    printf("Initializing IRLock on I2C\n");
    dev->register_periodic_callback(200, FUNCTOR_BIND_MEMBER(&AP_IRLock_I2C::read_frames, void));
}

/*
  converts IRLOCK pixels to a position on a normal plane 1m in front of the lens
  based on a characterization of IR-LOCK with the standard lens, focused such that 2.38mm of threads are exposed
 */
void AP_IRLock_I2C::pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y) {
    ret_x = (-0.00293875727162397f*pix_x + 0.470201163459835f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                                4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
    ret_y = (-0.003056843086277f*pix_y + 0.3056843086277f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                            4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
}

//void AP_IRLock_I2C::convert_pixy_data(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
//}


void AP_IRLock_I2C::read_frames(void) {
    uint8_t buf[16];
    dev->transfer(nullptr, 0, buf, 16);

    pixyObj.if_swap_buffer = 0;

    for (size_t i=0; i<16; i++) {
        pixyObj.recv_byte_pixy(buf[i]);
    }

                    // if (pixy parser just swapped buffers) {  ----------$$
                    //    take semaphore                    -----------$$
                    //    CONVERT frame
                    // copy frame data from pixy parser to target info array
                    //    update target_count
                    //    update frame_timestamp_us    ->    uint32_t timestamp_us = micros();
                    //    give semaphore
        // }
    

    //    printf("\n----IF_SWAP:  %d", pixyObj.if_swap_buffer);
    if (pixyObj.if_swap_buffer) {        
        if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {   //    take semaphore

            const pixy_parser::pixy_blob* temp;
            temp = pixyObj.read_buffer(0);

            if (temp != nullptr) {
                printf("\nBLOCK:- \nX: 0x%04x - Y: 0x%04x - W: 0x%04x - H: 0x%04x\n\n\n", temp->center_x, temp->center_y, temp->width, temp->height);

//                convert_pixy_data(temp->center_x, temp->center_y, temp->width, temp->height);   //    CONVERT frame

//---------------------
                int16_t corner1_pix_x = temp->center_x - temp->width/2;
                int16_t corner1_pix_y = temp->center_y - temp->height/2;
                int16_t corner2_pix_x = temp->center_x + temp->width/2;
                int16_t corner2_pix_y = temp->center_y + temp->height/2;
                float corner1_pos_x, corner1_pos_y, corner2_pos_x, corner2_pos_y;
                pixel_to_1M_plane(corner1_pix_x, corner1_pix_y, corner1_pos_x, corner1_pos_y);
                pixel_to_1M_plane(corner2_pix_x, corner2_pix_y, corner2_pos_x, corner2_pos_y);
//---------------------

                _target_info[target_count].pos_x = 0.5f*(corner1_pos_x+corner2_pos_x);      // copy frame data from pixy parser to target info array
                _target_info[target_count].pos_y = 0.5f*(corner1_pos_y+corner2_pos_y);
                _target_info[target_count].size_x = corner2_pos_x-corner1_pos_x;
                _target_info[target_count].size_x = corner2_pos_y-corner1_pos_y;

                target_count++;     //    update target_count
                
                _target_info[target_count].timestamp = AP_HAL::micros();       //    update frame_timestamp_us

                sem->give();    //    give semaphore

            } 
        }
    }   


/*
                    copy_pixy_data_to_target();
                }
            }            
        }

*/

//}
//    printf("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&------------------OUT");
//    printf("\nBLOCK:- \nX: 0x%04x - Y: 0x%04x - W: 0x%04x - H: 0x%04x\n\n\n", temp->center_x, temp->center_y, temp->width, temp->height);


//
//    }
//


}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_I2C::update() {
    bool new_data = false;
    if (!dev || !sem) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_last_update_ms != _target_info[0].timestamp) {
            new_data = true;
        }
        _last_update_ms = _target_info[0].timestamp;
        _flags.healthy = (AP_HAL::millis() - _last_read_ms < 100);
        sem->give();
    }
    // return true if new data found
    return new_data;
}