
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * pixy_parser.cpp
 * *
 */

#ifdef PIXY_PARSER_USE_ASSERT
#define PIXY_PARSER_ASSERT(x) {assert(x);}
#else
#define PIXY_PARSER_ASSERT(x) {}
#endif

#include "pixy_parser.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>


// Constructor - pixy_parser
pixy_parser::pixy_parser() {
    pixy_len = 0;
    blob_buffer_write_idx = 0;
    blob_buffer[blob_buffer_write_idx].count = 0;
}

// Method - empty_pixyBuf
void pixy_parser::empty_pixyBuf() {
    for (size_t i=0; i<pixy_len; i++) {
        pixy_buf[i] = 0;
    }
    pixy_len = 0;

    // printf("\n--------------------------AFTER EMPTY: ");
    for (size_t i=0; i<pixy_len; i++) {
        // printf("%u, ", (unsigned)pixy_buf[i]);
    }
}

// Method - print_buffer
void pixy_parser::print_buffer() {
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    // printf("Buffer:");
//    // printf("[%u, %u, %u, %u], ", (unsigned)writebuf.blobs[writebuf.count].center_x, (unsigned)writebuf.blobs[writebuf.count].center_y, (unsigned)writebuf.blobs[writebuf.count].width, (unsigned)writebuf.blobs[writebuf.count].height);
    for (size_t i=0; i<writebuf.count; i++) {
        // printf("[%u, %u, %u, %u], ", (unsigned)writebuf.blobs[i].center_x, (unsigned)writebuf.blobs[i].center_y, (unsigned)writebuf.blobs[i].width, (unsigned)writebuf.blobs[i].height);
    }
    // printf("   -   Count: %u\n", (unsigned)writebuf.count);
}

// Method - write_buffer
bool pixy_parser::write_buffer(const pixy_blob& received_blob) {
    // printf("Writing Main Buffer: Writing to %u\n", blob_buffer_write_idx);
//    struct blob_buffer* writebuf = &blob_buffer[blob_buffer_write_idx];
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    if (writebuf.count >= PIXY_PARSER_MAX_BLOBS) {
        return false;
    }
    writebuf.blobs[writebuf.count] = received_blob;
    // printf("SEE THIS----   [%u, %u, %u, %u], ", (unsigned)received_blob.center_x, (unsigned)received_blob.center_y, (unsigned)received_blob.width, (unsigned)received_blob.height);
    writebuf.count++;
    print_buffer();
    return true;
}

// Method - swap_buffer
// Blob buffer indexing swap
void pixy_parser::swap_buffer() {
    blob_buffer_write_idx = (blob_buffer_write_idx+1)%2;
    // printf("Swapping Main Buffer: Swapped to %u\n", blob_buffer_write_idx);
//    struct blob_buffer* writebuf = &blob_buffer[blob_buffer_write_idx];
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    writebuf.count = 0;        //This is probably wront. But helps me get rid of an error!!!-----------------------------------------------------------
    print_buffer();
}

// Method - read_buffer
// - read blob i:
 bool pixy_parser::read_buffer(size_t i, pixy_parser::pixy_blob& ret) {
    // printf("Reading Main Buffer: Reading from %u\n", (blob_buffer_write_idx+1)%2);
//    struct blob_buffer* readbuf = &blob_buffer[(blob_buffer_write_idx+1)%2];
    struct blob_buffer& readbuf = blob_buffer[(blob_buffer_write_idx+1)%2];
    if ( i >= PIXY_PARSER_MAX_BLOBS || i >= readbuf.count) {
        return false;
    }
    ret = readbuf.blobs[i];
    return true;
}

// Method - check_pixy_message
// Checks Validity of a inpyt byte stream
enum pixy_parser::message_validity_t pixy_parser::check_pixy_message() {
//    // printf("INSIDE CHECK_PIXY_MSG:%u\n", (unsigned)pixy_len);

    if (pixy_len == 0) {
        return MESSAGE_EMPTY;
    }
    if (pixy_len >= 1 && pixy_buf[0] != 0x55) {   //checking if 1st byte of the message is 0x55
        // printf("First block not 0x55 .. Message Invalid!\n");
        return MESSAGE_INVALID;
    }
    if (pixy_len >= 2 && pixy_buf[1] != 0xAA) {   //checking if 2nd byte of the message is 0xAA   (Combined it verifies if the message is 0x55AA)
        // printf("Second block not 0xAA .. Message Invalid!\n");
        return MESSAGE_INVALID;
    }
    if (pixy_len < 14) {     // Looks for at least 14 bytes i.e. "sync+blob_data" = 2+12 bytes)
        // printf("Message Length less than 14.. Message Incomplete!\n");
        return MESSAGE_INCOMPLETE;
    }

    if (pixy_buf[2] == 0x55 && pixy_buf[3] == 0xAA) {

        // printf("2 syncs available and size >= 14..\n");

        if (pixy_len >= 16) {

            // printf("PixyLen Greater than or equal to 16 with 2 syncs, calculating crc....\n");
            /* check crc */
            uint16_t crc_calculated = 0;    //addition of all the message fields from 5 to 15
            for (size_t i = 6; i <= 15; i+=2) {
                uint16_t word = pixy_buf[i] | (uint16_t)pixy_buf[i+1]<<8;
                crc_calculated += word;
            }

            uint16_t crc_provided;  //Getting true crc by concatinating 4 & 5 message fields(original crc) into a single 16 bit
            crc_provided = pixy_buf[4] | (uint16_t)pixy_buf[5]<<8;
            if (crc_provided != crc_calculated) {
                // printf("CRC Failed Message Invalid!\n");
                return MESSAGE_INVALID;
            }
            return MESSAGE_VALID_SOF;
        } else {
            return MESSAGE_INCOMPLETE;
        }


    } else {
       // printf("Only 1 SYNC available and size < 16, calculating CRC....\n");

        uint16_t crc_calculated = 0;    //addition of all the message fields from 5 to 15
        for (size_t i = 4; i <= 13; i+=2) {
            uint16_t word = pixy_buf[i] | (uint16_t)pixy_buf[i+1]<<8;
            crc_calculated += word;
        }
        uint16_t crc_provided;  //Getting true crc by concatinating 4 & 5 message fields(original crc) into a single 16 bit
        crc_provided = pixy_buf[2] | (uint16_t)pixy_buf[3]<<8;
        // printf("Crc calculated: %u Crc provided: %u\n", (unsigned)crc_calculated, (unsigned)crc_provided);

        if (crc_provided != crc_calculated) {
            // printf("CRC Failed Message Invalid!\n");
            return MESSAGE_INVALID;
        }
        return MESSAGE_VALID_BLOCK;
    }
}

// Method - recv_byte_pixy
// Logicbase of Pixy_parser. Receives the byte from here and adds into pixyBuf if valid.
bool pixy_parser::recv_byte_pixy(uint8_t byte) {
//    // printf("INSIDE recv_byte_pixy:-\n");
// Read 2 bytes
//    printf("\n----  IF_SWAP BEFORE:  %d", if_swap_buffer);

    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];


    // printf("The byte inputted is: %u and pixylen is %u\n", byte, (unsigned)pixy_len);

    PIXY_PARSER_ASSERT(pixy_len < PIXY_PARSER_PIXY_BUF_SIZE);
    if (pixy_len >= PIXY_PARSER_PIXY_BUF_SIZE) {
        // This should never happen.
        empty_pixyBuf();
    }

    pixy_buf[pixy_len++] = byte;        // Append byte to buffer

    enum message_validity_t validity = check_pixy_message();    // Call parser
    // printf("Validity Check Call: %d (0:empty 1:invalid 2:incomplete 3:valid_SOF 4:valid_block)\n", validity);
    // printf("Pixy Len: %u\n", (unsigned)pixy_len);
    // printf("Pixy_buf: ");
//    for (size_t i=0; i<PIXY_PARSER_PIXY_BUF_SIZE; i++) {
        // printf("%u, ", pixy_buf[i]);
//    }
    // printf("\n");

    if (validity == MESSAGE_VALID_SOF) {
        if (!(writebuf.count >= PIXY_PARSER_MAX_BLOBS)) {
            pixy_blob received_blob;
            received_blob.center_x = pixy_buf[8] | (uint16_t)pixy_buf[9]<<8;    //Extract blob information
            received_blob.center_y = pixy_buf[10] | (uint16_t)pixy_buf[11]<<8;
            received_blob.width = pixy_buf[12] | (uint16_t)pixy_buf[13]<<8;
            received_blob.height = pixy_buf[14] | (uint16_t)pixy_buf[15]<<8;

            if (writebuf.count != 0) {
                swap_buffer();  //swap buffer
                if_swap_buffer = 1; 

                writebuf.count = 0;     //Turn the count to 0
                // printf("@@@SEE THIS----   [%u, %u, %u, %u], ", (unsigned)received_blob.center_x, (unsigned)received_blob.center_y, (unsigned)received_blob.width, (unsigned)received_blob.height);

                write_buffer(received_blob);  //Writing inside the buffer(the count increment happens inside the write_buffer())
                return true;
            }
            else {
                write_buffer(received_blob);    //Writing inside the buffer(the count increment happens inside the write_buffer())
            }
        }
        empty_pixyBuf();
    }

    if (validity == MESSAGE_VALID_BLOCK) {     ///-----------------------------how can I store the block inside a frame?-----------------
        if (!(writebuf.count >= PIXY_PARSER_MAX_BLOBS)) {
            pixy_blob received_blob;
            received_blob.center_x = pixy_buf[6] | (uint16_t)pixy_buf[7]<<8; //Extract blob information
            received_blob.center_y = pixy_buf[8] | (uint16_t)pixy_buf[9]<<8;
            received_blob.width = pixy_buf[10] | (uint16_t)pixy_buf[11]<<8;
            received_blob.height = pixy_buf[12] | (uint16_t)pixy_buf[13]<<8;
            // printf("@@@SEE THIS----   [%u, %u, %u, %u], ", (unsigned)received_blob.center_x, (unsigned)received_blob.center_y, (unsigned)received_blob.width, (unsigned)received_blob.height);

            write_buffer(received_blob);  //Writing inside the buffer
        }
        empty_pixyBuf();    //Emptying The pixy_buf once read the SOF into 'writebuf'       
    }

    if (validity == MESSAGE_INVALID) {  // If message invalid, wait and read 2 bytes
        pixy_len--;     ///----------------------------------------QUES----------------------------------------------
        memmove(pixy_buf, pixy_buf+1, pixy_len);    // This discards the first block of the pixy_buf and makes pixy_buf have everything except the fisrt block value
        validity = check_pixy_message();

        if (writebuf.count < PIXY_PARSER_MAX_BLOBS && writebuf.count != 0) {
            swap_buffer();  //swap buffer
            if_swap_buffer = 1;
            return true;
        }
    }
    return false;
}
