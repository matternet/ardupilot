#ifndef __MESSAGES_H
#define __MESSAGES_H

#include <stdint.h>

#ifndef FTS_PACKED
#define FTS_PACKED __attribute__((packed))
#endif

#define FTS_DEPLOY_MAGIC 0x5555

enum fts_msg_id_t {
    FTS_MSGID_UNKNOWN=0,
    FTS_MSGID_STATUS=1,
    FTS_MSGID_DEPLOY=2,
    FTS_MSGID_WDRST=3,
    FTS_MSGID_POWER_OFF=4,
    FTS_MSGID_ARM_CMD=5,
    FTS_MSGID_DISARM_CMD=6,
    FTS_MSGID_VERSION=7,
    FTS_MSGID_STATUS2=8
};

enum fts_state_t {
    FTS_INIT=0,
    FTS_POWER_OFF=1,
    FTS_DISARMED_MASTER_NOT_PRESENT=2,
    FTS_DISARMED_MASTER_PRESENT=3,
    FTS_ARMED=4,
    FTS_DEPLOYED=5
};

enum fts_state_reason_t {
    FTS_REASON_INIT=0,
    FTS_REASON_COMMANDED=1,
    FTS_REASON_WATCHDOG_EXPIRE=2,
    FTS_REASON_BUTTON_PRESS=3
};

enum fuse_fault_state_t {
    FTS_FUSE_STATE_UNKNOWN=0,
    FTS_FUSE_STATE_PASS=1,
    FTS_FUSE_STATE_FAIL=2
};

struct fts_msg_status_s {
    uint8_t msgid;
    uint8_t state;
    uint8_t state_reason;
    uint16_t batt_cell_mV[3];
    uint8_t batt_SoC;
    int16_t batt_temp_TS1;
    int16_t batt_temp_INT;
} FTS_PACKED;

struct fts_msg_deploy_s {
    uint8_t msgid;
    uint16_t magic;
} FTS_PACKED;

struct fts_msg_wdrst_s {
    uint8_t msgid;
} FTS_PACKED;

struct fts_msg_power_off_s {
    uint8_t msgid;
} FTS_PACKED;

struct fts_msg_arm_cmd_s {
    uint8_t msgid;
} FTS_PACKED;

struct fts_msg_disarm_cmd_s {
    uint8_t msgid;
} FTS_PACKED;

struct fts_msg_version_s {
    uint8_t msgid;
    uint8_t git_hash[7];
} FTS_PACKED;

struct fts_msg_status2_s {
    uint8_t msgid;
    int16_t wdt_min_margin_ms;
    uint8_t fuse_fault_state;
} FTS_PACKED;

#endif
