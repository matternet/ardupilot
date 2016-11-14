#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include "messages.h"

uint8_t fts_protocol_rx_byte(uint8_t byte, uint8_t ret_buf_size, uint8_t* ret_buf);
enum fts_msg_id_t fts_protocol_identify_message(uint8_t msg_len, uint8_t* msg);
uint8_t fts_protocol_encode_message(uint8_t in_buf_size, uint8_t* in_buf, uint8_t out_buf_size, uint8_t* out_buf);

#endif
