
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
#define PIXY_PARSER_MAX_BLOBS 10

class pixy_parser {
    public:
        typedef struct {
            uint16_t center_x;
            uint16_t center_y;
            uint16_t width;
            uint16_t height;
        } pixy_blob;

        pixy_parser();
        void empty_pixyBuf(void);
        void print_buffer(void);
        void swap_buffer(void);
        bool recv_byte_pixy(uint8_t byte);
        bool read_buffer(size_t i, pixy_blob& ret);

    private:

        struct blob_buffer {    //Frame (full of blobs)
            pixy_blob blobs[PIXY_PARSER_MAX_BLOBS];
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
        enum message_validity_t check_pixy_message();

        uint8_t pixy_buf[PIXY_PARSER_PIXY_BUF_SIZE];
        size_t pixy_len;
        uint8_t blob_buffer_write_idx;
};


     








