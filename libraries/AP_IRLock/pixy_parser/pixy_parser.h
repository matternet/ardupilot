/*
 * pixy_parser.h
 *
 */

#pragma once

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define PIXY_PARSER_PIXY_BUF_SIZE 17

/*
    pixy_buf_size = 17;
    pixy_buf[17]= {};
    pixy_len = 0;
    blob_buffer_write_idx = 0;
    bytes_to_sof = 0;
    bytes_to_block = 0;
}
*/
class pixy_parser {
    public:
        pixy_parser();
        ~pixy_parser();
        void empty_pixyBuf(void);
        void print_buffer(void);
        void swap_buffer(void);
        void recv_byte_pixy(uint8_t byte);

    private:
        typedef struct {
            uint16_t center_x;
            uint16_t center_y;
            uint16_t width;
            uint16_t height;
        } pixy_blob;

        struct blob_buffer {    //Frame (full of blobs)
            pixy_blob blobs[10];
            size_t count;
        } blob_buffer[2];


        enum message_validity_t {
            MESSAGE_EMPTY,
            MESSAGE_INVALID,
            MESSAGE_INCOMPLETE,
            MESSAGE_VALID_SOF,
            MESSAGE_VALID_BLOCK
        };

        bool write_buffer(const pixy_blob& blob1);
        const pixy_blob* read_buffer(size_t i);
        enum message_validity_t check_pixy_message(size_t pixy_len);

        uint8_t pixy_buf[PIXY_PARSER_PIXY_BUF_SIZE];
        size_t pixy_len;
        uint8_t blob_buffer_write_idx;
        size_t bytes_to_sof;
        size_t bytes_to_block;
};











