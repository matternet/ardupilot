#include "pixy_parser.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


int main() {
    pixy_parser pixy;

    uint8_t input_bytes[] = {0x55, 0xAA, 0x55, 0xAA, 0x0F, 0x01, 0x01, 0x00, 0x68, 0x00, 0x52, 0x00, 0x24, 0x00, 0x30, 0x00, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x55, 0xAA, 0x17, 0x01, 0x01, 0x00, 0x69, 0x00, 0x56, 0x00, 0x27, 0x00, 0x30, 0x00, 0x55, 0xAA, 0x17, 0x01, 0xF1, 0xE0, 0x09, 0x10, 0x46, 0x00, 0x27, 0xE0, 0x00, 0xF0, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x00, 0x01, 0xF1, 0x0A, 0xE0, 0x10, 0x55, 0x00, 0x15, 0x11, 0x2F, 0x00, 0x55, 0xAA, 0x55, 0xAA, 0x0F, 0x01, 0x01, 0x00, 0x68, 0x00, 0x52, 0x00, 0x24, 0x00, 0x30, 0x00};  //array of 15 bytes of SOF 
//    uint8_t input_bytes[] = {85,  170,  85,   170,  15,   1,    1,    0,    104,   0,    82,   0,   36,    0,   48,   0,    85,  170,  144,   1,     1,    0,   229,   0,   86,    0,   37,   0,    47,    0,   85,   170,   85,  170,  23,    1,    1,   0,    105,  0,     86,   0,    39,   0,   48,    0,   85,   170,  23,   1,   241,  224,   9,    16,   70,    0,   39,   224,  0,   240,  124   };  //array of 15 bytes of SOF 

    size_t size_input_bytes = sizeof(input_bytes)/sizeof(input_bytes[0]);

    for (size_t i=0; i<=size_input_bytes; i++) {
        printf("%u, ", (unsigned)input_bytes[i]);
        }

    uint8_t input_bytes2[] = {};
    for (size_t i=0; i<=size_input_bytes; i++) {
//        printf("Calling recv_byte_pixy : %d       :  %u\n", i, (unsigned)input_bytes[i]);
        printf("\nNEW BYTE IN:\n");
        pixy.recv_byte_pixy(input_bytes[i]);
    }
    return 0;
}