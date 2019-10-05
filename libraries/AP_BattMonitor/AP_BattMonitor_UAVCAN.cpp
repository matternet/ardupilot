#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"
#include "AP_BattMonitor_Analog_Table.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define debug_bm_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return;
    }

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        switch (_type) {
            case UAVCAN_BATTERY_INFO:
                if (ap_uavcan->register_BM_bi_listener_to_id(this, _params._serial_number)) {
                    debug_bm_uavcan(2, "UAVCAN BattMonitor BatteryInfo registered id: %d\n\r", _params._serial_number);
                }
                break;
        }
    }
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    if (batt_changed) {
        batt_changed = false;
        float soc_pct = AP_BattMonitor_Analog_Table::lookup_SoC_table(_state.voltage);
        reset_remaining(soc_pct);
        Write_DataFlash_Log_Startup_messages();
    }
    
    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

void AP_BattMonitor_UAVCAN::handle_bi_msg(float voltage, float current, float temperature, uint32_t _model_instance_id, uint8_t _model_name[33])
{
    _state.temperature = temperature;
    _state.voltage = voltage;
    _state.current_amps = current;

    batt_changed = strncmp((const char *)model_name, (const char *)_model_name, sizeof(model_name)) != 0;

    model_instance_id = _model_instance_id;
    memcpy(model_name, _model_name, sizeof(model_name));

    uint32_t tnow = AP_HAL::micros();
    uint32_t dt = tnow - _state.last_time_micros;

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000) {
        // .0002778 is 1/3600 (conversion to hours)
        float mah = (float) ((double) _state.current_amps * (double) dt * (double) 0.0000002778f);
        _state.consumed_mah += mah;
        _state.consumed_wh  += 0.001f * mah * _state.voltage;
    }

    // record time
    _state.last_time_micros = tnow;

    _state.healthy = true;
}

void AP_BattMonitor_UAVCAN::Write_DataFlash_Log_Startup_messages()
{
    gcs().send_text(MAV_SEVERITY_INFO, "BMU Inst=%u Model=%s", unsigned(model_instance_id), (const char *)model_name);
}

#endif
