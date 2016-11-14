#include "protocol.h"
#include "slip.h"

#include <string.h>

#define FTS_PROTOCOL_RXBUF_SIZE 32

static uint8_t rxbuf[FTS_PROTOCOL_RXBUF_SIZE];
static uint8_t rxbuf_len = 0;
static uint8_t wait_for_end_byte = 0;

static uint16_t crc16_ccitt(uint8_t byte, uint16_t crc);

enum fts_msg_id_t fts_protocol_identify_message(uint8_t msg_len, uint8_t* msg) {
    switch (msg[0]) {
        case FTS_MSGID_STATUS:
            if (msg_len != sizeof(struct fts_msg_status_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_DEPLOY:
            if (msg_len != sizeof(struct fts_msg_deploy_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_WDRST:
            if (msg_len != sizeof(struct fts_msg_wdrst_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_POWER_OFF:
            if (msg_len != sizeof(struct fts_msg_power_off_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_ARM_CMD:
            if (msg_len != sizeof(struct fts_msg_arm_cmd_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_DISARM_CMD:
            if (msg_len != sizeof(struct fts_msg_disarm_cmd_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_VERSION:
            if (msg_len != sizeof(struct fts_msg_version_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        case FTS_MSGID_STATUS2:
            if (msg_len != sizeof(struct fts_msg_status2_s)) {
                return FTS_MSGID_UNKNOWN;
            }
            break;
        default:
            return FTS_MSGID_UNKNOWN;
    }
    return (enum fts_msg_id_t)msg[0];
}



// accepts a byte, appends it to rxbuf
// if a full frame has been received, writes the payload to ret_buf.
// if data was written, returns pointer to outbuf in ret_buf
// returns length of valid data written to outbuf
uint8_t fts_protocol_rx_byte(uint8_t byte, uint8_t ret_buf_size, uint8_t* ret_buf) {
    uint8_t i;
    uint8_t decoded_buf[FTS_PROTOCOL_RXBUF_SIZE-1];
    uint8_t decoded_len;
    uint16_t crc16_provided;
    uint16_t crc16_computed;

    if (rxbuf_len >= FTS_PROTOCOL_RXBUF_SIZE) {
        // no room in rxbuf, clear it and flag the rxbuf as misaligned unless byte received was an end byte
        rxbuf_len = 0;
        if (byte != SLIP_END) {
            wait_for_end_byte = 1;

        }
        return 0;
    }

    if (wait_for_end_byte) {
        wait_for_end_byte = (byte != SLIP_END);
        return 0;
    }

    rxbuf[rxbuf_len++] = byte;

    if (byte == SLIP_END) {
        decoded_len = slip_decode(rxbuf_len, rxbuf, decoded_buf);
        rxbuf_len = 0;

        if (decoded_len-3 > ret_buf_size) {
            return 0;
        }

        if (decoded_len <= 3) {
            return 0;
        }

        if (decoded_buf[0] != decoded_len-3) {
            return 0;
        }

        // little-endian
        crc16_provided = 0;
        crc16_provided |= decoded_buf[decoded_len-1];
        crc16_provided <<= 8;
        crc16_provided |= decoded_buf[decoded_len-2];

        crc16_computed = 0;
        for (i=0; i < decoded_len-2; i++) {
            crc16_computed = crc16_ccitt(decoded_buf[i], crc16_computed);
        }

        if (crc16_provided != crc16_computed) {
            return 0;
        }

        for (i=0; i < decoded_len-3; i++) {
            ret_buf[i] = decoded_buf[i+1];
        }

        return decoded_len-3;
    }

    return 0;
}

// returns 0 on failure, 1 on success
uint8_t fts_protocol_encode_message(uint8_t in_buf_size, uint8_t* in_buf, uint8_t out_buf_size, uint8_t* out_buf) {
    uint8_t out_len;
    uint8_t i;
    uint16_t crc16_val;
    out_len = 0;
    crc16_val = 0;

    if (!slip_encode_and_append(in_buf_size, &out_len, out_buf, out_buf_size)) { return 0; }
    crc16_val = crc16_ccitt(in_buf_size, crc16_val);

    for (i=0; i<in_buf_size; i++) {
        if (!slip_encode_and_append(in_buf[i], &out_len, out_buf, out_buf_size)) { return 0; }
        crc16_val = crc16_ccitt(in_buf[i], crc16_val);
    }


    if (!slip_encode_and_append((crc16_val) & 0xff, &out_len, out_buf, out_buf_size)) { return 0; }
    if (!slip_encode_and_append((crc16_val >> 8) & 0xff, &out_len, out_buf, out_buf_size)) { return 0; }

    if (out_len >= out_buf_size) { return 0; }
    out_buf[out_len++] = SLIP_END;

    return out_len;
}

static uint16_t crc16_ccitt(uint8_t byte, uint16_t crc16)
{
    uint8_t i;
    crc16 ^= (byte << (16 - 8));
    for (i = 8; i > 0; i--)
    {
        if (crc16 & 0x8000)
        {
            crc16 = (crc16 << 1) ^ 0x1021;
        }
        else
        {
            crc16 = (crc16 << 1);
        }
    }

    return crc16;
}
