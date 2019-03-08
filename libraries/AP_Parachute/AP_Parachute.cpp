#include "AP_Parachute.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>

#include <GCS_MAVLink/GCS.h>
#include "protocol.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_Parachute, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is not released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 5, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),
    
    AP_GROUPEND
};

char AP_Parachute::_mttr_fts_version[16];

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;
    
    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _release_in_progress = true;
            _released = true;
        }
    }else if ((_release_time == 0) || time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }


    if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_MATTERNET_FTS) {
        mttr_fts_update();
    }
}

void AP_Parachute::mttr_fts_transmit(uint8_t msg_len, uint8_t* msg_buf)
{
    if (!_mttr_uart) {
        return;
    }

    uint8_t encoded_msg[32];
    uint8_t encoded_msg_len = fts_protocol_encode_message(msg_len, msg_buf, 32, encoded_msg);
    if (_mttr_uart->txspace() >= encoded_msg_len) {
        _mttr_uart->write(encoded_msg, encoded_msg_len);
    }
}

void AP_Parachute::send_debug_message(uint32_t tnow_ms, uint8_t ind, float value)
{
    mavlink_message_t mav_msg;
    mavlink_debug_t mav_dbg_msg;
    mav_dbg_msg.time_boot_ms = tnow_ms;
    mav_dbg_msg.ind = ind;
    mav_dbg_msg.value = value;
    mavlink_msg_debug_encode(mavlink_system.sysid, mavlink_system.compid, &mav_msg, &mav_dbg_msg);
    GCS_MAVLINK::send_on_all_channels(&mav_msg);
}

void AP_Parachute::mttr_fts_update()
{
    if (!_mttr_uart) {
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    // 20Hz
    if (tnow_ms - _mttr_last_loop_ms > 50) {
        if (_release_in_progress || _released) {
            struct fts_msg_deploy_s deploy_msg;
            deploy_msg.msgid = FTS_MSGID_DEPLOY;
            deploy_msg.magic = FTS_DEPLOY_MAGIC;
            mttr_fts_transmit(sizeof(deploy_msg), (uint8_t*)&deploy_msg);
        }

        if (tnow_ms - _mttr_last_status_recv_ms > 1000) {
            _mttr_status_pass = false;
        }

        struct fts_msg_wdrst_s wdrst_msg;
        wdrst_msg.msgid = FTS_MSGID_WDRST;
        mttr_fts_transmit(sizeof(wdrst_msg), (uint8_t*)&wdrst_msg);

        if (!_release_initiated) {
            if (hal.util->get_soft_armed()) {
                struct fts_msg_arm_cmd_s arm_msg;
                arm_msg.msgid = FTS_MSGID_ARM_CMD;
                mttr_fts_transmit(sizeof(arm_msg), (uint8_t*)&arm_msg);
            } else {
                struct fts_msg_disarm_cmd_s disarm_msg;
                disarm_msg.msgid = FTS_MSGID_DISARM_CMD;
                mttr_fts_transmit(sizeof(disarm_msg), (uint8_t*)&disarm_msg);
            }
        }

        _mttr_last_loop_ms = tnow_ms;
    }

    uint8_t msg_buf[32];
    int16_t byte;

    while ((byte = _mttr_uart->read()) != -1) {
        uint8_t msg_len = fts_protocol_rx_byte(byte, 32, msg_buf);
        if (msg_len > 0) {
            enum fts_msg_id_t msg_id = fts_protocol_identify_message(msg_len, msg_buf);
            if (msg_id == FTS_MSGID_STATUS) {
                struct fts_msg_status_s* msg = (struct fts_msg_status_s*)msg_buf;
                DataFlash_Class::instance()->Log_Write("FTSS", "TimeUS,State,Rsn,BSoC,BC1mV,BC2mV,BC3mV,BTINT,BTTS1", "QBBBHHHcc", AP_HAL::micros64(), msg->state, msg->state_reason, msg->batt_SoC, msg->batt_cell_mV[0], msg->batt_cell_mV[1], msg->batt_cell_mV[2], msg->batt_temp_INT, msg->batt_temp_TS1);

                // update pre-arm state
                _mttr_last_status_recv_ms = tnow_ms;
                _mttr_status_pass = (msg->state == FTS_DISARMED_MASTER_PRESENT || msg->state == FTS_ARMED);

                // send via MAVLink
                send_debug_message(tnow_ms, 0, msg->state);
                send_debug_message(tnow_ms, 1, msg->state_reason);
                send_debug_message(tnow_ms, 2, msg->batt_SoC);
                send_debug_message(tnow_ms, 3, msg->batt_cell_mV[0]*1e-3f);
                send_debug_message(tnow_ms, 4, msg->batt_cell_mV[1]*1e-3f);
                send_debug_message(tnow_ms, 5, msg->batt_cell_mV[2]*1e-3f);
                send_debug_message(tnow_ms, 6, msg->batt_temp_INT*1e-2f);
                send_debug_message(tnow_ms, 7, msg->batt_temp_TS1*1e-2f);

            } else if (msg_id == FTS_MSGID_VERSION) {
                struct fts_msg_version_s* msg = (struct fts_msg_version_s*)msg_buf;
                memset(_mttr_fts_version, 0, sizeof(_mttr_fts_version));
                memcpy(_mttr_fts_version, msg->git_hash, sizeof(msg->git_hash));
                DataFlash_Class::instance()->Log_Write("FTSV", "TimeUS,Hash", "QN", AP_HAL::micros64(), _mttr_fts_version);
            } else if (msg_id == FTS_MSGID_STATUS2) {
                struct fts_msg_status2_s* msg = (struct fts_msg_status2_s*)msg_buf;

                _mttr_fuse_pass = (msg->fuse_fault_state == FTS_FUSE_STATE_PASS);

                DataFlash_Class::instance()->Log_Write("FTS2", "TimeUS,Fuse,WDTm", "QBh", AP_HAL::micros64(), msg->fuse_fault_state, msg->wdt_min_margin_ms);

                // send via MAVLink
                send_debug_message(tnow_ms, 8, msg->fuse_fault_state);
                send_debug_message(tnow_ms, 9, msg->wdt_min_margin_ms);
            }
        }
    }
}
