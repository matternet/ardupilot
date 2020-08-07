/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for UAVCAN IMUs using the ardupilot::imu::FastIMU message
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class FastIMUCb;

class AP_InertialSensor_UAVCAN : public AP_InertialSensor_Backend {
public:
    void start() override;
    bool update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_InertialSensor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_InertialSensor_Backend* probe(AP_InertialSensor &ins);

    static void handle_FastIMU(AP_UAVCAN* ap_uavcan, uint8_t node_id, const FastIMUCb &cb);

private:
    AP_InertialSensor_UAVCAN(AP_InertialSensor &imu);

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    uint16_t sample_time_us;
    
    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN *ap_uavcan;
        uint8_t node_id;
        AP_InertialSensor_UAVCAN *driver;
    } _detected_modules[INS_MAX_INSTANCES];

    static HAL_Semaphore _sem_registry;

    uint8_t accel_instance;
    uint8_t gyro_instance;
};
