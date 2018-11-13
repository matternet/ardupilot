#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>

#define PIXY_BUF_SIZE 17

enum message_validity_t {
    MESSAGE_EMPTY,
    MESSAGE_INVALID,
    MESSAGE_INCOMPLETE,
    MESSAGE_VALID_SOF,
    MESSAGE_VALID_BLOCK
};

static uint8_t pixy_buf[PIXY_BUF_SIZE];
static size_t pixy_len;
static uint8_t blob_buffer_write_idx;
static size_t bytes_to_sof;
static size_t bytes_to_block;

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


void empty_pixyBuf() {
    for (size_t i=0; i<pixy_len; i++) {
        pixy_buf[i] = 0;    
    }
    pixy_len = 0;
}


void print_buffer() {
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    printf("Buffer:");  
//    printf("[%u, %u, %u, %u], ", (unsigned)writebuf.blobs[writebuf.count].center_x, (unsigned)writebuf.blobs[writebuf.count].center_y, (unsigned)writebuf.blobs[writebuf.count].width, (unsigned)writebuf.blobs[writebuf.count].height);
    for (size_t i=0; i<writebuf.count; i++) {
        printf("[%u, %u, %u, %u], ", (unsigned)writebuf.blobs[i].center_x, (unsigned)writebuf.blobs[i].center_y, (unsigned)writebuf.blobs[i].width, (unsigned)writebuf.blobs[i].height);
    }
    printf("   -   Count: %u\n", (unsigned)writebuf.count);
}


// - Write Buffer
bool write_buffer(const pixy_blob& blob1) {
    printf("Writing Main Buffer: Writing to %u\n", blob_buffer_write_idx);
//    struct blob_buffer* writebuf = &blob_buffer[blob_buffer_write_idx];
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    if (writebuf.count >= 10) {
        return false;
    }
    writebuf.blobs[writebuf.count] = blob1;
    printf("SEE THIS----   [%u, %u, %u, %u], ", (unsigned)blob1.center_x, (unsigned)blob1.center_y, (unsigned)blob1.width, (unsigned)blob1.height);
    writebuf.count++;
    print_buffer();
    return true;
}

// Blob buffer indexing swap
void swap_buffer() {
    blob_buffer_write_idx = (blob_buffer_write_idx+1)%2;
    printf("Swapping Main Buffer: Swapped to %u\n", blob_buffer_write_idx);
//    struct blob_buffer* writebuf = &blob_buffer[blob_buffer_write_idx];
    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];
    writebuf.count = 0;        //This is probably wront. But helps me get rid of an error!!!-----------------------------------------------------------
    print_buffer();
}


// - read blob i:

const pixy_blob* read_buffer(size_t i) {
    printf("Reading Main Buffer: Reading from %u\n", (blob_buffer_write_idx+1)%2);
//    struct blob_buffer* readbuf = &blob_buffer[(blob_buffer_write_idx+1)%2];
    struct blob_buffer& readbuf = blob_buffer[(blob_buffer_write_idx+1)%2];
    if (i >= readbuf.count) {
        return NULL;  
    } 
    return &readbuf.blobs[i];
}

static enum message_validity_t check_pixy_message(size_t pixy_len) {
//    printf("INSIDE CHECK_PIXY_MSG:%u\n", (unsigned)pixy_len);

    if (pixy_len == 0) {
        return MESSAGE_EMPTY;
    }
    if (pixy_len >= 1 && pixy_buf[0] != 0x55) {   //checking if 1st byte of the message is 0x55
        printf("First block not 0x55 .. Message Invalid!\n");
        return MESSAGE_INVALID;
    }
    if (pixy_len >= 2 && pixy_buf[1] != 0xAA) {   //checking if 2nd byte of the message is 0xAA   (Combined it verifies if the message is 0x55AA)
        printf("Second block not 0xAA .. Message Invalid!\n");
        return MESSAGE_INVALID;
    }
    if (pixy_buf[2] == 0x55 && pixy_buf[3] == 0xAA) {
        bytes_to_sof = 16 - pixy_len;
        if (bytes_to_sof == 0) {
            printf("SOF COMPLETE!\n");    
        } 
        else {
            printf("Need %u more bytes to complete a SOF\n", (unsigned)bytes_to_sof);
        }
    }
    if (pixy_buf[2] != 0x55 && pixy_buf[3] != 0xAA) {
        bytes_to_block = 14 - pixy_len;
        if (bytes_to_block == 0) {
            printf("BLOCK COMPLETE!\n");    
        } 
        else {
            printf("Need %u more bytes to complete a Block\n", (unsigned)bytes_to_sof);
        }
    }
    if (pixy_len < 14) {     // Looks for at least 14 bytes i.e. "sync+blob_data" = 2+12 bytes)  
        printf("Message Length less than 14.. Message Incomplete!\n");
        return MESSAGE_INCOMPLETE;  
    }

    if (pixy_buf[2] == 0x55 && pixy_buf[3] == 0xAA) {    

        printf("2 syncs available and size >= 14..\n");

        if (pixy_len >= 16) {    

            printf("PixyLen Greater than or equal to 16 with 2 syncs, calculating crc....\n");
            /* check crc */
            uint16_t crc_calculated = 0;    //addition of all the message fields from 5 to 15 
            for (size_t i = 6; i <= 15; i+=2) {
                uint16_t word = pixy_buf[i] | (uint16_t)pixy_buf[i+1]<<8; 
                crc_calculated += word;
            }

            uint16_t crc_provided;  //Getting true crc by concatinating 4 & 5 message fields(original crc) into a single 16 bit
            crc_provided = pixy_buf[4] | (uint16_t)pixy_buf[5]<<8;
            if (crc_provided != crc_calculated) {
                printf("CRC Failed Message Invalid!\n");
                return MESSAGE_INVALID;
            }            
            return MESSAGE_VALID_SOF;
        } else {
            return MESSAGE_INCOMPLETE;  
        }


    } else {
       printf("Only 1 SYNC available and size < 16, calculating CRC....\n");

        uint16_t crc_calculated = 0;    //addition of all the message fields from 5 to 15 
        for (size_t i = 4; i <= 13; i+=2) {
            uint16_t word = pixy_buf[i] | (uint16_t)pixy_buf[i+1]<<8; 
            crc_calculated += word;
        }
        uint16_t crc_provided;  //Getting true crc by concatinating 4 & 5 message fields(original crc) into a single 16 bit
        crc_provided = pixy_buf[2] | (uint16_t)pixy_buf[3]<<8;        
        printf("Crc calculated: %u Crc provided: %u\n", (unsigned)crc_calculated, (unsigned)crc_provided);

        if (crc_provided != crc_calculated) {
            printf("CRC Failed Message Invalid!\n");
            return MESSAGE_INVALID;
        }
        return MESSAGE_VALID_BLOCK;
    }
}

void recv_byte_pixy(uint8_t byte) {
//    printf("INSIDE recv_byte_pixy:-\n");

// Read 2 bytes
    pixy_blob blob1;    //%%%%%%%%%%%%%%%%%%%%$$$$$$$$$$$$$$$$$$$$$$$$-------------- should I put struct as prefix or not?-------------------------%%%%%%%%%$$$$$$$$$

    struct blob_buffer& writebuf = blob_buffer[blob_buffer_write_idx];

    printf("The byte inputted is: %u and pixylen is %u\n", byte, (unsigned)pixy_len);
    pixy_buf[pixy_len++] = byte;        // Append byte to buffer

    enum message_validity_t validity = check_pixy_message(pixy_len);    // Call parser 
    printf("Validity Check Call: %d (0:empty 1:invalid 2:incomplete 3:valid_SOF 4:valid_block)\n", validity);
    printf("Pixy Len: %u\n", (unsigned)pixy_len);
    printf("Pixy_buf: ");   
    for (size_t i=0; i<17; i++) {
        printf("%u, ", pixy_buf[i]);
    }   
    printf("\n");

    if (validity == MESSAGE_VALID_SOF) {
//        pixy_frame_received(pixy_len, pixy_buf);
        
        if (!(writebuf.count >= 10)) {
            if (writebuf.count != 0) {
                swap_buffer();  //swap buffer

                blob1.center_x = pixy_buf[8] | (uint16_t)pixy_buf[9]<<8;    //Extract blob information
                blob1.center_y = pixy_buf[10] | (uint16_t)pixy_buf[11]<<8;
                blob1.width = pixy_buf[12] | (uint16_t)pixy_buf[13]<<8;
                blob1.height = pixy_buf[14] | (uint16_t)pixy_buf[15]<<8;

                writebuf.count = 0;     //Turn the count to 0
                printf("@@@SEE THIS----   [%u, %u, %u, %u], ", (unsigned)blob1.center_x, (unsigned)blob1.center_y, (unsigned)blob1.width, (unsigned)blob1.height);

                write_buffer(blob1);  //Writing inside the buffer(the count increment happens inside the write_buffer())
                empty_pixyBuf();    //Emptying The pixy_buf once read the SOF into 'writebuf'

            }
            else {
                write_buffer(blob1);    //Writing inside the buffer(the count increment happens inside the write_buffer())
                empty_pixyBuf();    //Emptying The pixy_buf once read the SOF into 'writebuf'

                printf("\n--------------------------AFTER EMPTY: ");
                for (int i=0; i<pixy_len; i++) {
                    printf("%u, ", (unsigned)pixy_buf[i]);
                }
            }
        }
    }

    if (validity == MESSAGE_VALID_BLOCK) {     ///-----------------------------how can I store the block inside a frame?-----------------
        if (!(writebuf.count >= 10)) {
            blob1.center_x = pixy_buf[6] | (uint16_t)pixy_buf[7]<<8; //Extract blob information
            blob1.center_x = pixy_buf[8] | (uint16_t)pixy_buf[9]<<8; 
            blob1.width = pixy_buf[10] | (uint16_t)pixy_buf[11]<<8;
            blob1.height = pixy_buf[12] | (uint16_t)pixy_buf[13]<<8;
            printf("@@@SEE THIS----   [%u, %u, %u, %u], ", (unsigned)blob1.center_x, (unsigned)blob1.center_y, (unsigned)blob1.width, (unsigned)blob1.height);

            write_buffer(blob1);  //Writing inside the buffer
            empty_pixyBuf();    //Emptying The pixy_buf once read the SOF into 'writebuf'

            printf("\n------------------------------AFTER EMPTY: ");
            for (int i=0; i<pixy_len; i++) {
                printf("%u, ", (unsigned)pixy_buf[i]);
            }

        }
    }

    if (validity == MESSAGE_INVALID) {  // If message invalid, wait and read 2 bytes
        pixy_len--;     ///----------------------------------------QUES----------------------------------------------
        memmove(pixy_buf, pixy_buf+1, pixy_len);    // This discards the first block of the pixy_buf and makes pixy_buf have everything except the fisrt block value
        validity = check_pixy_message(pixy_len);

        if (writebuf.count < 10 && writebuf.count != 0) {
            swap_buffer();  //swap buffer
        }        
    }    
}

int main() {
    uint8_t input_bytes[] = {0x55, 0xAA, 0x55, 0xAA, 0x0F, 0x01, 0x01, 0x00, 0x68, 0x00, 0x52, 0x00, 0x24, 0x00, 0x30, 0x00, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x55, 0xAA, 0x17, 0x01, 0x01, 0x00, 0x69, 0x00, 0x56, 0x00, 0x27, 0x00, 0x30, 0x00, 0x55, 0xAA, 0x17, 0x01, 0xF1, 0xE0, 0x09, 0x10, 0x46, 0x00, 0x27, 0xE0, 0x00, 0xF0, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x90, 0x01, 0x01, 0x00, 0xE5, 0x00, 0x56, 0x00, 0x25, 0x00, 0x2F, 0x00, 0x55, 0xAA, 0x00, 0x01, 0xF1, 0x0A, 0xE0, 0x10, 0x55, 0x00, 0x15, 0x11, 0x2F, 0x00, 0x55, 0xAA, 0x55, 0xAA, 0x0F, 0x01, 0x01, 0x00, 0x68, 0x00, 0x52, 0x00, 0x24, 0x00, 0x30, 0x00};  //array of 15 bytes of SOF 
//    uint8_t input_bytes[] = {85,  170,  85,   170,  15,   1,    1,    0,    104,   0,    82,   0,   36,    0,   48,   0,    85,  170,  144,   1,     1,    0,   229,   0,   86,    0,   37,   0,    47,    0,   85,   170,   85,  170,  23,    1,    1,   0,    105,  0,     86,   0,    39,   0,   48,    0,   85,   170,  23,   1,   241,  224,   9,    16,   70,    0,   39,   224,  0,   240,  124   };  //array of 15 bytes of SOF 

    for (size_t i=0; i<=sizeof(input_bytes); i++) {
        printf("%u, ", (unsigned)input_bytes[i]);
    }

    uint8_t input_bytes2[] = {};
    for (size_t i=0; i<=sizeof(input_bytes); i++) {
//        printf("Calling recv_byte_pixy : %d       :  %u\n", i, (unsigned)input_bytes[i]);
        printf("\nNEW BYTE IN:\n");
        recv_byte_pixy(input_bytes[i]);
    }
    return 0;
}