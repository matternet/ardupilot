/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

class AP_ESC_Telem {
public:
    AP_ESC_Telem();

    /* Do not allow copies */
    AP_ESC_Telem(const AP_ESC_Telem &other) = delete;
    AP_ESC_Telem &operator=(const AP_ESC_Telem&) = delete;

    void init(void);

    void send_esc_telemetry_mavlink(uint8_t chan);

    static AP_ESC_Telem *get_singleton(void) {
        return singleton;
    }

private:
    static AP_ESC_Telem *singleton;
    AP_HAL::UARTDriver *uart;

    void timer_update();

    struct PACKED {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 0x16
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        uint16_t load;
        uint16_t current;
        uint16_t temperature;
        uint16_t unknown;
        uint16_t crc;
    } pkt;

    bool initialised;
    uint8_t len;
    uint32_t last_read_ms;

    struct {
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        float voltage;
        uint16_t load;
        float current;
        uint16_t temperature;
        uint16_t unknown;
    } decoded;

    uint32_t last_mavlink_ms;

    AP_HAL::Semaphore *sem;

    void parse_packet(void);
};
