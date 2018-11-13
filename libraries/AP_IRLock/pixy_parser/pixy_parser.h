/*
 * pixy_parser.h
 *
 */

#pragma once
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#define PIXY_BUF_SIZE 17
    

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

        static struct blob_buffer {    //Frame (full of blobs)   
            pixy_blob blobs[10];
            size_t count;
        } blob_buffer[2];

        bool write_buffer(const pixy_blob& blob1);
    
        const pixy_blob* read_buffer(size_t i);

        uint8_t pixy_buf[PIXY_BUF_SIZE];
        size_t pixy_len;
        uint8_t blob_buffer_write_idx;
        size_t bytes_to_sof;
        size_t bytes_to_block;


};











