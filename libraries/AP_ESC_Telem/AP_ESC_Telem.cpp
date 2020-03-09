/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */
#include "AP_ESC_Telem.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0x9B
#define TELEM_LEN    0x16

AP_ESC_Telem *AP_ESC_Telem::singleton;

// constructor
AP_ESC_Telem::AP_ESC_Telem(void)
{
    singleton = this;
}

void AP_ESC_Telem::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_instance();
    if (!serial_manager) {
        return;
    }
    uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry, 0);
    if (uart) {
        sem = hal.util->new_semaphore();
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_ESC_Telem::timer_update, void));
    }
}

/*
  update ESC telemetry
 */
void AP_ESC_Telem::timer_update()
{
    if (!initialised) {
        initialised = true;
        uart->begin(19200);
    }

    uint32_t n = uart->available();
    if (n == 0) {
        return;
    }

    // we expect at least 50ms idle between frames
    uint32_t now = AP_HAL::millis();
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    // don't read too much in one loop to prevent too high CPU load
    n = MIN(n, 500U);
    if (len == 0 && !frame_gap) {
        // discard
        while (n--) {
            uart->read();
        }
        return;
    }

    if (frame_gap) {
        len = 0;
    }

    while (n--) {
        uint8_t b = uart->read();
        //hal.console->printf("t=%u 0x%02x\n", now, b);
        if (len == 0 && b != TELEM_HEADER) {
            continue;
        }
        if (len == 1 && b != TELEM_LEN) {
            continue;
        }
        uint8_t *buf = (uint8_t *)&pkt;
        buf[len++] = b;
        if (len == sizeof(pkt)) {
            parse_packet();
            len = 0;
        }
    }
}

static uint16_t calc_crc(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc += *buf++;
    }
    return crc;
}

/*
  parse packet
 */
void AP_ESC_Telem::parse_packet(void)
{
    uint16_t crc = calc_crc((uint8_t *)&pkt, sizeof(pkt)-2);
    if (crc != pkt.crc) {
        return;
    }

    sem->take_blocking();
    decoded.counter = be32toh(pkt.counter);
    decoded.throttle_req = be16toh(pkt.throttle_req);
    decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = be16toh(pkt.erpm) * 5.0 / 7.0;
    decoded.voltage = be16toh(pkt.voltage) * 0.1;
    decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    decoded.phase_current = int16_t(be16toh(pkt.phase_current)) * 0.01;
    decoded.temperature = be16toh(pkt.temperature);
    decoded.status = be16toh(pkt.status);
    sem->give();

#if 0
    hal.console->printf("%u RPM:%.0f THR:%u:%u V:%.2f PC:%.2f C:%.2f T:0x%x S:0x%x\n",
                        unsigned(decoded.counter),
                        decoded.rpm,
                        unsigned(decoded.throttle_req), unsigned(decoded.throttle),
                        decoded.voltage,
                        decoded.phase_current,
                        decoded.current,
                        unsigned(decoded.temperature),
                        unsigned(decoded.status));
#endif

    DataFlash_Class::instance()->Log_Write("HESC", "TimeUS,CNT,RPM,ThrR,Thr,Volt,CurrP,Curr,Temp,Status",
                                           "QIfHHfffHH",
                                           AP_HAL::micros64(),
                                           decoded.counter,
                                           decoded.rpm,
                                           decoded.throttle_req, decoded.throttle,
                                           decoded.voltage,
                                           decoded.phase_current,
                                           decoded.current,
                                           decoded.temperature,
                                           decoded.status);
}

/*
  send telemetry on mavlink
 */
void AP_ESC_Telem::send_esc_telemetry_mavlink(uint8_t chan)
{
    if (!uart) {
        return;
    }
    uint8_t temperature[4] {};
    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, ESC_TELEMETRY_1_TO_4)) {
        return;
    }
    sem->take_blocking();
    voltage[0] = decoded.voltage * 1000;
    current[0] = decoded.current;
    rpm[0] = decoded.rpm;
    sem->give();
    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)chan, temperature, voltage, current, totalcurrent, rpm, count);
}
