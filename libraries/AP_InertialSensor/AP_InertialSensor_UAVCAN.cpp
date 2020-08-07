#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_InertialSensor_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <ardupilot/imu/FastIMU.hpp>

extern const AP_HAL::HAL& hal;

#define BACKEND_SAMPLE_RATE 2000

// UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(FastIMUCb, ardupilot::imu::FastIMU);

AP_InertialSensor_UAVCAN::DetectedModules AP_InertialSensor_UAVCAN::_detected_modules[] = {0};

HAL_Semaphore AP_InertialSensor_UAVCAN::_sem_registry;

/*
  constructor
 */
AP_InertialSensor_UAVCAN::AP_InertialSensor_UAVCAN(AP_InertialSensor &ins) :
    AP_InertialSensor_Backend(ins)
{}

/*
  subscribe to UAVCAN messages, called in AP_UAVCAN startup
 */
void AP_InertialSensor_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto *node = ap_uavcan->get_node();

    uavcan::Subscriber<ardupilot::imu::FastIMU, FastIMUCb> *FastIMU_listener;
    FastIMU_listener = new uavcan::Subscriber<ardupilot::imu::FastIMU, FastIMUCb>(*node);

    // Msg Handler
    const int FastIMU_listener_res = FastIMU_listener->start(FastIMUCb(ap_uavcan, &handle_FastIMU));
    if (FastIMU_listener_res < 0) {
        AP_HAL::panic("UAVCAN InertialSensor subscriber start fail");
        return;
    }
}

AP_InertialSensor_Backend* AP_InertialSensor_UAVCAN::probe(AP_InertialSensor &ins)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_InertialSensor_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_InertialSensor_UAVCAN(ins);
            if (backend) {
                _detected_modules[i].driver = backend;
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;
            }
            break;
        }
    }
    return backend;
}

/*
  find a matching backend for a node_id
 */
AP_InertialSensor_UAVCAN* AP_InertialSensor_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        // Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}

/*
  handler for incoming FastIMU messages
 */
void AP_InertialSensor_UAVCAN::handle_FastIMU(AP_UAVCAN* ap_uavcan, uint8_t node_id, const FastIMUCb &cb)
{
    AP_InertialSensor_UAVCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_uavcan_backend(ap_uavcan, node_id, true);
        if (driver == nullptr) {
            return;
        }
    }
    {
        auto *msg = cb.msg;
        static uint8_t last_counter;
        if ((last_counter+1) % 256 != msg->counter) {
            ::printf("FastIMU lost %u %u\n", last_counter, msg->counter);
        }
        last_counter = msg->counter;

        Vector3f accel(msg->accel[0], msg->accel[1], msg->accel[2]);
        accel *= (10.0 * (1+msg->accel_scale)) / INT16_MAX;

        Vector3f gyro(msg->gyro[0], msg->gyro[1], msg->gyro[2]);
        gyro *= float(1+msg->gyro_scale) / INT16_MAX;

        WITH_SEMAPHORE(driver->_sem);
        driver->sample_time_us = msg->time_us;

        driver->_rotate_and_correct_accel(driver->accel_instance, accel);
        driver->_notify_new_accel_raw_sample(driver->accel_instance, accel);

        driver->_rotate_and_correct_gyro(driver->gyro_instance, gyro);
        driver->_notify_new_gyro_raw_sample(driver->gyro_instance, gyro);
    }
}

// Read the sensor
bool AP_InertialSensor_UAVCAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_UAVCAN::start()
{
    if (sample_time_us != 0) {
        uint32_t devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                     _ap_uavcan->get_driver_index(),
                                                     _node_id,
                                                     0);
        ::printf("registering UAVCAN IMU for %u\n", _node_id);
        accel_instance = _imu.register_accel(1e6/sample_time_us, devid);
        gyro_instance = _imu.register_gyro(1e6/sample_time_us,  devid);
    }
}


#endif // HAL_WITH_UAVCAN

